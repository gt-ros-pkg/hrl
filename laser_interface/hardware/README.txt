This hardware design was made using Eagle CAD (sch and brd included).
The Gerber files (top, bot, drd, slk, bsk, oln, smt, smb) can be sent
to any board house for manufacture.  I prefer 4pcb.com.  The BOM is
also contained herein with (correct?) part numbers.

In a nutshell, this board hooks into standard computer mice, drawing
whatever battery power is available (roughly 1.8-5.0V), conditioning
it (to roughly 3.2V to drive a green laser diode), and then activating
the greed diode (laser connection) when the mouse button is pressed.
It requires three connections to the mouse: Power (batt+), Ground
(batt-), and switch (active-low on the mouse button's DPST switch).

That is all.
