/* size sweep for variable-length SysEx over output_direct: at what length does delivery break?
 * Prints the byte counts actually received in order, to see truncation vs. late delivery. */
#include <alsa/asoundlib.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

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

static int recv_one(snd_seq_t *seq, unsigned char *out, int max, int timeout_ms, int quiet)
{
    struct pollfd fds[4];
    if (snd_seq_poll_descriptors(seq, fds, 4, POLLIN) <= 0) return -1;
    if (poll(fds, 1, timeout_ms) <= 0) return -2;
    snd_seq_event_t *ev = NULL;
    int r = snd_seq_event_input(seq, &ev);
    if (r < 0) { if (!quiet) printf("  input err %d\n", r); return -3; }
    if (ev->type != SND_SEQ_EVENT_SYSEX) { if (!quiet) printf("  type %d (not sysex)\n", ev->type); snd_seq_free_event(ev); return -4; }
    int len = ev->data.ext.len;
    if (!quiet) printf("  got ext.len=%d first=%02x last=%02x\n", len, ((unsigned char *)ev->data.ext.ptr)[0],
                       ((unsigned char *)ev->data.ext.ptr)[len - 1]);
    if (len > max) len = max;
    memcpy(out, ev->data.ext.ptr, len);
    snd_seq_free_event(ev);
    return len;
}

int main(void)
{
    snd_seq_t *tx = NULL, *rx = NULL;
    unsigned char buf[1024], got[2048];
    int t, r;

    snd_seq_open(&tx, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK);
    snd_seq_set_client_name(tx, "SB-TX");
    t = snd_seq_create_simple_port(tx, "SysEx Out", SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
                                   SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    snd_seq_open(&rx, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK);
    snd_seq_set_client_name(rx, "SB-RX");
    r = snd_seq_create_simple_port(rx, "SysEx In", SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
                                   SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    snd_seq_connect_from(rx, r, snd_seq_client_id(tx), t);

    static const int sizes[] = {6, 64, 100, 127, 128, 129, 200, 300, 600, 1000};
    for (unsigned k = 0; k < sizeof sizes / sizeof sizes[0]; k++) {
        int n = sizes[k];
        memset(buf, 0, sizeof buf);
        buf[0] = 0xF0; buf[n - 1] = 0xF7;
        for (int i = 1; i < n - 1; i++) buf[i] = (unsigned char)(i & 0x7F);
        int sent = send_sysex(tx, t, buf, n);
        int gotn = recv_one(rx, got, sizeof got, 1500, 0);
        int ok = gotn == n && !memcmp(got, buf, n);
        printf("len %4d sent %3d -> got %4d  %s\n", n, sent, gotn, ok ? "exact" : "MISMATCH");
        if (!ok) {
            int extra = recv_one(rx, got, sizeof got, 300, 0);   /* is the rest waiting? */
            printf("     next read: %d\n", extra);
        }
    }
    snd_seq_close(rx);
    snd_seq_close(tx);
    return 0;
}
