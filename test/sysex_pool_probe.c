/* Are the missing events queued but unread (probe bug) or actually dropped?
 * Drains with a zero-timeout input loop instead of poll-per-event, and logs the client's
 * input pool room from /proc/asound/seq/clients before sending, after sending, after reading. */
#include <alsa/asoundlib.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void pools(const char *when, int rx_client)
{
    FILE *f = fopen("/proc/asound/seq/clients", "r");
    if (!f) return;
    char line[256];
    int in_client = 0;
    printf("-- %s\n", when);
    while (fgets(line, sizeof line, f)) {
        int c;
        if (sscanf(line, "Client %d", &c) == 1) in_client = (c == rx_client);
        if (in_client && (strstr(line, "Pool size") || strstr(line, "Cells in use") || strstr(line, "Client ")))
            printf("   %s", line);
    }
    fclose(f);
}

static int send_direct(snd_seq_t *seq, int port, const unsigned char *buf, int len)
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

int main(void)
{
    snd_seq_t *tx = NULL, *rx = NULL;
    unsigned char buf[8];
    int t, r;

    snd_seq_open(&tx, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK);
    snd_seq_set_client_name(tx, "SB-TX");
    t = snd_seq_create_simple_port(tx, "SysEx Out", SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
                                   SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    snd_seq_open(&rx, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK);
    snd_seq_set_client_name(rx, "SB-RX");
    r = snd_seq_create_simple_port(rx, "SysEx In", SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
                                   SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    int rx_client = snd_seq_client_id(rx);
    snd_seq_connect_from(rx, r, snd_seq_client_id(tx), t);

    pools("after subscribe (rx client above)", rx_client);

    buf[0] = 0xF0; buf[2] = 0xF7;
    for (int i = 0; i < 20; i++) { buf[1] = (unsigned char)i; send_direct(tx, t, buf, 3); }
    pools("after 20 sends, before reading", rx_client);

    /* zero-timeout drain: poll first, then keep pulling until -EAGAIN */
    int got = 0, err = 0;
    for (;;) {
        snd_seq_event_t *ev = NULL;
        int rr = snd_seq_event_input(rx, &ev);
        if (rr == -EAGAIN || rr == -EWOULDBLOCK) break;
        if (rr < 0) { err = rr; break; }
        got++;
        snd_seq_free_event(ev);
    }
    printf("zero-timeout drain: %d events (err %d)\n", got, err);

    struct pollfd fds[4];
    int np = snd_seq_poll_descriptors(rx, fds, 4, POLLIN);
    int pr = poll(fds, np > 0 ? np : 1, 300);
    printf("poll after drain: ret=%d revents=0x%x (nothing more expected)\n", pr, pr > 0 ? fds[0].revents : 0);

    /* and one more send, read immediately: the pattern that worked */
    buf[1] = 0x7F;
    printf("late send: %d\n", send_direct(tx, t, buf, 3));
    int rr = 0;
    snd_seq_event_t *ev = NULL;
    struct pollfd f2[4];
    snd_seq_poll_descriptors(rx, f2, 4, POLLIN);
    poll(f2, 1, 500);
    rr = snd_seq_event_input(rx, &ev);
    printf("read after late send: %d\n", rr);
    if (rr >= 0) snd_seq_free_event(ev);

    snd_seq_disconnect_from(rx, r, snd_seq_client_id(tx), t);
    snd_seq_close(rx);
    snd_seq_close(tx);
    return 0;
}
