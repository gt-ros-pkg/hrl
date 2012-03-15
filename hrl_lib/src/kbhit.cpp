// .cpp

#ifdef _WIN32
#else


#include "hrl_lib/kbhit.h"

static struct termios termattr, save_termattr;
static int ttysavefd = -1;
static enum 
{ 
  RESET, RAW, CBREAK 
} ttystate = RESET;

/* ***************************************************************************
 *
 * set_tty_raw(), put the user's TTY in one-character-at-a-time mode.
 * returns 0 on success, -1 on failure.
 *
 *************************************************************************** */
int
set_tty_raw(void) 
{
  int i;

  i = tcgetattr (STDIN_FILENO, &termattr);
  if (i < 0) 
  {
    printf("tcgetattr() returned %d for fildes=%d\n",i,STDIN_FILENO); 
    perror ("");
    return -1;
  }
  save_termattr = termattr;

  termattr.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  termattr.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  termattr.c_cflag &= ~(CSIZE | PARENB);
  termattr.c_cflag |= CS8;
  termattr.c_oflag &= ~(OPOST);
   
  termattr.c_cc[VMIN] = 1;  /* or 0 for some Unices;  see note 1 */
  termattr.c_cc[VTIME] = 0;

  i = tcsetattr (STDIN_FILENO, TCSANOW, &termattr);
  if (i < 0) 
  {
    printf("tcsetattr() returned %d for fildes=%d\n",i,STDIN_FILENO); 
    perror("");
    return -1;
  }
   
  ttystate = RAW;
  ttysavefd = STDIN_FILENO;

  return 0;
}

/* ***************************************************************************
 *
 * set_tty_cbreak(), put the user's TTY in cbreak mode.
 * returns 0 on success, -1 on failure.
 *
 *************************************************************************** */
int 
set_tty_cbreak() 
{
  int i;

  i = tcgetattr (STDIN_FILENO, &termattr);
  if (i < 0) 
  {
    printf("tcgetattr() returned %d for fildes=%d\n",i,STDIN_FILENO); 
    perror ("");
    return -1;
  }

  save_termattr = termattr;

  termattr.c_lflag &= ~(ECHO | ICANON);
  termattr.c_cc[VMIN] = 1;
  termattr.c_cc[VTIME] = 0;
      
  i = tcsetattr (STDIN_FILENO, TCSANOW, &termattr);
  if (i < 0) 
  {
    printf("tcsetattr() returned %d for fildes=%d\n",i,STDIN_FILENO); 
    perror ("");
    return -1;
  }
  ttystate = CBREAK;
  ttysavefd = STDIN_FILENO;

  return 0;
}

/* ***************************************************************************
 *
 * set_tty_cooked(), restore normal TTY mode. Very important to call
 *   the function before exiting else the TTY won't be too usable.
 * returns 0 on success, -1 on failure.
 *
 *************************************************************************** */
int
set_tty_cooked() 
{
  int i;
  if (ttystate != CBREAK && ttystate != RAW) 
  {
    return 0;
  }
  i = tcsetattr (STDIN_FILENO, TCSAFLUSH, &save_termattr);
  if (i < 0) 
  {
    return -1;
  }
  ttystate = RESET;
  return 0;
}

/* ***************************************************************************
 *
 * kb_getc(), if there's a typed character waiting to be read,
 *   return it; else return 0.
 * 10-sep-2006: kb_getc() fails (it hangs on the read() and never returns
 * until a char is typed) under some Unix/Linux versions: ubuntu, suse, and
 * maybe others. To make it work, please uncomment two source lines below.
 *
 *************************************************************************** */
unsigned char
kb_getc(void) 
{
  int i;
  unsigned char ch;
  ssize_t size;
  termattr.c_cc[VMIN] = 0; /* uncomment if needed */
  i = tcsetattr (STDIN_FILENO, TCSANOW, &termattr);
  size = read (STDIN_FILENO, &ch, 1);
  termattr.c_cc[VMIN] = 1; /* uncomment if needed */
  i = tcsetattr (STDIN_FILENO, TCSANOW, &termattr);
  if (size == 0)
  {
    return 0;
  }
  else
  {
    return ch;
  }
}

/* ***************************************************************************
 *
 * kb_getc_w(), wait for a character to be typed and return it.
 *
 *************************************************************************** */
unsigned char
kb_getc_w(void) 
{
  unsigned char ch;
  size_t size;

  while (1)
  {

    usleep(20000);        /* 1/50th second: thanks, Floyd! */

    size = read (STDIN_FILENO, &ch, 1);
    if (size > 0)
    {
      break;
    }
  }
  return ch;
}


#endif // _WIN32

