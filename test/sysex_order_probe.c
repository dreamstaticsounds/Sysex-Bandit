/* Which message arrives when? Tag every send (second byte = its id) and log the arrival order with
 * timestamps, including the no-subscriber send, to find reordering/late delivery. */
#include <alsa/asoundlib.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static int send_sysex(snd_seq_t *seq, int port, const unsigned char *buf, int len)
{
    snd_seq_event_t ev;
    snd_seq_ev_clear(&ev);
    snd_seq_ev_set_source(&ev, port);
    snd_seq_ev_set_subs(&ev);
    snd_seq_ev_set_direct(&ev);
    ev.type = SND_SEQ_EVENT_SYSEX;
    snd_seq_ev_set_variable(&ev, len, (void *)buf);
    return snd_seq_event_output_direct(seq, &ev);
}

static int recv_one(snd_seq_t *seq, int timeout_ms)
{
    struct pollfd fds[4];
    if (snd_seq_poll_descriptors(seq, fds, 4, POLLIN) <= 0) return -1;
    if (poll(fds, 1, timeout_ms) <= 0) return -2;
    snd_seq_event_t *ev = NULL;
    int r = snd_seq_event_input(seq, &ev);
    if (r < 0) return -3;
    if (ev->type != SND_SEQ_EVENT_SYSEX) {
        printf("  [%8.1f] type=%d (ignored)\n", now_ms(), ev->type);
        snd_seq_free_event(ev);
        return -4;
    }
    int len = ev->data.ext.len;
    unsigned char *b = (unsigned char *)ev->data.ext.ptr;
    printf("  [%8.1f] len=%4d tag=%02x first=%02x last=%02x\n", now_ms(), len,
           len > 1 ? b[1] : 0, b[0], b[len - 1]);
    snd_seq_free_event(ev);
    return len;
}

int main(void)
{
    snd_seq_t *tx = NULL, *rx = NULL;
    unsigned char buf[1024];
    int t, r;
    double t0;

    snd_seq_open(&tx, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK);
    snd_seq_set_client_name(tx, "SB-TX");
    t = snd_seq_create_simple_port(tx, "SysEx Out", SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
                                   SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    snd_seq_open(&rx, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK);
    snd_seq_set_client_name(rx, "SB-RX");
    r = snd_seq_create_simple_port(rx, "SysEx In", SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
                                   SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);

    /* A: sent with no subscriber */
    memset(buf, 0, sizeof buf);
    buf[0] = 0xF0; buf[1] = 0x0A; buf[2] = 0xF7;
    t0 = now_ms();
    printf("send A (no subs): %d\n", send_sysex(tx, t, buf, 3));

    snd_seq_connect_from(rx, r, snd_seq_client_id(tx), t);
    printf("subscribed at %.1f\n", now_ms() - t0);

    /* B */
    buf[1] = 0x0B;
    printf("send B: %d\n", send_sysex(tx, t, buf, 3));
    printf("recv after B:"); recv_one(rx, 2000);

    /* big */
    memset(buf, 0, sizeof buf);
    buf[0] = 0xF0; buf[1] = 0xEE; buf[499] = 0xF7;
    for (int i = 2; i < 499; i++) buf[i] = (unsigned char)(i & 0x7F);
    printf("send big(500): %d\n", send_sysex(tx, t, buf, 500));
    printf("recv after big:"); recv_one(rx, 2000);

    /* D, E */
    buf[1] = 0x0D; printf("send D: %d\n", send_sysex(tx, t, buf, 3));
    buf[1] = 0x0E; printf("send E: %d\n", send_sysex(tx, t, buf, 3));
    printf("recv after D+E #1:"); recv_one(rx, 2000);
    printf("recv after D+E #2:"); recv_one(rx, 2000);
    printf("recv after D+E #3:"); recv_one(rx, 2000);

    snd_seq_disconnect_from(rx, r, snd_seq_client_id(tx), t);
    snd_seq_close(rx);
    snd_seq_close(tx);
    return 0;
}
