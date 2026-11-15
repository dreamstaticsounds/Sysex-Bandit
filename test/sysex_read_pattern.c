/* Confirm the reader-loop trap: after ONE snd_seq_event_input, does poll() still signal while
 * events remain buffered? (alsa-lib pulls a batch from the fd into its own buffer.)
 * Correct pattern = poll once, then drain with a zero-timeout input loop until -EAGAIN. */
#include <alsa/asoundlib.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

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

static int readable(snd_seq_t *seq, int timeout_ms)
{
    struct pollfd fds[4];
    if (snd_seq_poll_descriptors(seq, fds, 4, POLLIN) <= 0) return -1;
    return poll(fds, 1, timeout_ms) > 0;
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
    snd_seq_connect_from(rx, r, snd_seq_client_id(tx), t);

    buf[0] = 0xF0; buf[2] = 0xF7;
    for (int i = 0; i < 5; i++) { buf[1] = (unsigned char)i; send_direct(tx, t, buf, 3); }

    printf("poll before any read:        %d (want 1)\n", readable(rx, 200));

    snd_seq_event_t *ev = NULL;
    int rr = snd_seq_event_input(rx, &ev);
    printf("read one event:              %d\n", rr);
    if (rr >= 0) snd_seq_free_event(ev);

    printf("poll after reading one,      %d (traps the naive poll-per-event loop if 0)\n", readable(rx, 300));

    int rest = 0;
    for (;;) {
        ev = NULL;
        rr = snd_seq_event_input(rx, &ev);
        if (rr < 0) break;
        rest++;
        snd_seq_free_event(ev);
    }
    printf("zero-timeout drain after it: %d events still waiting (want 4)\n", rest);
    printf("poll now (drained):          %d (want 0)\n", readable(rx, 200));

    snd_seq_disconnect_from(rx, r, snd_seq_client_id(tx), t);
    snd_seq_close(rx);
    snd_seq_close(tx);
    return 0;
}
