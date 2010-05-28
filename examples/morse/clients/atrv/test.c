#include <stdio.h>
#include <curses.h>

int main(int argc, char *argv[])
{
    char c;
    initscr();
    cbreak();
    do
    {
        c = getch();
    } while (c != 'a');
    endwin();
    return 0;
}
