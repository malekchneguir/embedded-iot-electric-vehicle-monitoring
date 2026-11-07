/**
 * porttcp.c — Port Modbus TCP pour STM32H753ZI + LwIP + FreeRTOS
 *
 * Architecture :
 *   - Réception : callback LwIP Raw (dans tcpip_thread) → flag + événement FreeModbus
 *   - Envoi     : tcpip_callback() → exécution dans tcpip_thread (seule façon safe)
 */

#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "mbtcp.h"

#include <string.h>
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "cmsis_os.h"

#define MODBUS_TCP_PORT      502
#define MODBUS_TCP_BUF_SIZE  260

/* ---- Variables ----------------------------------------------------------- */
static struct tcp_pcb *pxServerPCB = NULL;
static struct tcp_pcb *pxClientPCB = NULL;

/* Buffer de réception (écrit par tcpip_thread, lu par defaultTask) */
static uint8_t  ucRxBuf[MODBUS_TCP_BUF_SIZE];
static uint16_t usRxLen = 0;
static volatile BOOL xFrameReady = FALSE;

/* Buffer de réponse (écrit par defaultTask, lu par tcpip_thread via callback) */
static uint8_t  ucTxBuf[MODBUS_TCP_BUF_SIZE];
static uint16_t usTxLen = 0;

/* Sémaphore pour attendre la fin de l'envoi */
static osSemaphoreId_t xTxDoneSem = NULL;

/* ---- Callback d'envoi — exécuté par tcpip_thread ------------------------- */
static void prvDoSend(void *arg)
{
    (void)arg;
    if (pxClientPCB != NULL && usTxLen > 0)
    {
        tcp_write(pxClientPCB, ucTxBuf, usTxLen, TCP_WRITE_FLAG_COPY);
        tcp_output(pxClientPCB);
    }
    /* Signale à defaultTask que l'envoi est lancé */
    osSemaphoreRelease(xTxDoneSem);
}

/* ---- Callbacks TCP (appelés depuis tcpip_thread) ------------------------- */
static err_t prvOnReceive(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    (void)arg;
    if (p == NULL) {
        tcp_close(tpcb);
        pxClientPCB = NULL;
        return ERR_OK;
    }
    if (err != ERR_OK) { pbuf_free(p); return err; }

    uint16_t len = (p->tot_len > MODBUS_TCP_BUF_SIZE) ? MODBUS_TCP_BUF_SIZE : p->tot_len;
    pbuf_copy_partial(p, ucRxBuf, len, 0);
    usRxLen = len;
    xFrameReady = TRUE;

    /* Notifie eMBPoll() qu'une trame Modbus est disponible */
    xMBPortEventPost(EV_FRAME_RECEIVED);

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t prvOnAccept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg;
    if (err != ERR_OK) return err;
    if (pxClientPCB != NULL) { tcp_close(newpcb); return ERR_OK; }

    pxClientPCB = newpcb;
    tcp_setprio(newpcb, TCP_PRIO_NORMAL);
    tcp_recv(newpcb, prvOnReceive);
    return ERR_OK;
}

/* ---- API FreeModbus ------------------------------------------------------- */

BOOL xMBTCPPortInit(USHORT usTCPPort)
{
    /* Sémaphore binaire pour synchroniser l'envoi */
    xTxDoneSem = osSemaphoreNew(1, 0, NULL);
    if (xTxDoneSem == NULL) return FALSE;

    /* Créer le serveur TCP dans tcpip_thread via LOCK */
    LOCK_TCPIP_CORE();

    pxServerPCB = tcp_new();
    if (pxServerPCB == NULL) { UNLOCK_TCPIP_CORE(); return FALSE; }

    if (tcp_bind(pxServerPCB, IP_ADDR_ANY, MODBUS_TCP_PORT) != ERR_OK) {
        tcp_close(pxServerPCB); pxServerPCB = NULL;
        UNLOCK_TCPIP_CORE(); return FALSE;
    }

    pxServerPCB = tcp_listen(pxServerPCB);
    if (pxServerPCB == NULL) { UNLOCK_TCPIP_CORE(); return FALSE; }

    tcp_accept(pxServerPCB, prvOnAccept);

    UNLOCK_TCPIP_CORE();
    return TRUE;
}

void vMBTCPPortClose(void)
{
    LOCK_TCPIP_CORE();
    if (pxClientPCB) { tcp_close(pxClientPCB); pxClientPCB = NULL; }
    if (pxServerPCB) { tcp_close(pxServerPCB); pxServerPCB = NULL; }
    UNLOCK_TCPIP_CORE();
}

void vMBTCPPortDisable(void) { vMBTCPPortClose(); }

BOOL xMBTCPPortGetRequest(UCHAR **ppucMBTCPFrame, USHORT *usTCPLength)
{
    if (!xFrameReady) return FALSE;
    xFrameReady = FALSE;
    *ppucMBTCPFrame = ucRxBuf;
    *usTCPLength    = usRxLen;
    return TRUE;
}

BOOL xMBTCPPortSendResponse(const UCHAR *pucMBTCPFrame, USHORT usTCPLength)
{
    if (pxClientPCB == NULL || usTCPLength > MODBUS_TCP_BUF_SIZE) return FALSE;

    /* Copie la réponse dans le buffer de TX */
    memcpy(ucTxBuf, pucMBTCPFrame, usTCPLength);
    usTxLen = usTCPLength;

    /* Demande à tcpip_thread d'envoyer et attend la confirmation */
    if (tcpip_callback(prvDoSend, NULL) != ERR_OK) return FALSE;
    osSemaphoreAcquire(xTxDoneSem, osWaitForever);

    return TRUE;
}
