/* The engine's reader loop shape: poll(fd) -> read one event -> poll again. Does the fd signal
 * level-triggered (still readable with events left) or does it go quiet until new data arrives?
 * Logs revents per iteration. */
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

    for (int step = 0; step < 8; step++) {
        struct pollfd fds[4];
        int np = snd_seq_poll_descriptors(rx, fds, 4, POLLIN);
        int pr = poll(fds, 1, 200);
        short re = pr > 0 ? fds[0].revents : 0;
        int got = 0, err = 0;
        /* read everything currently readable, zero-timeout */
        for (;;) {
            snd_seq_event_t *ev = NULL;
            int rr = snd_seq_event_input(rx, &ev);
            if (rr == -EAGAIN) break;
            if (rr < 0) { err = rr; break; }
            got++;
            snd_seq_free_event(ev);
        }
        printf("step %d: fds=%d poll ret=%d revents=0x%x -> drained %d (err %d)\n",
               step, np, pr, re, got, err);
        if (pr <= 0 && got == 0) break;
    }

    snd_seq_disconnect_from(rx, r, snd_seq_client_id(tx), t);
    snd_seq_close(rx);
    snd_seq_close(tx);
    return 0;
}
