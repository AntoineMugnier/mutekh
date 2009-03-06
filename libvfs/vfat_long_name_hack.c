
#include <vfs/vfs.h>
#include <string.h>
#include <assert.h>

/* Process a path to be compliant with the 
 * stupid upper short names of VFAT.
 * 
 * This is a temporarily function waiting for
 * libVFS to support long filenames.
 *
 * 8 upper characters max before the extension:
 *  - blabla -> BLABLA
 *  - blablabla -> BLABLAB~1
 * 3 upper characters max for the extension:
 *  - blablabla.b -> BLABLAB~1.B
 *  - blabla.blabla -> BLABLA.BLA
 *
 *  This function expect 'new' to be a table of at least 13 char (8+1+3+1).
 */
void touppershortname(char* new, const char* path)
{
    /* sanity check */
    assert(path != NULL);
    if (*path == 0) return;

    const char *p = path;
    char *n = new;
    char *delim;
    delim = strchr(path, '.');

    size_t i;

    /* process the body name */
    for (i = 0; i < 8 && *p && (p != delim); i++, p++, n++)
    {
        if ((delim - path) > 8 && i >= 6)
        {
            if (i == 6)
                *n = '~';
            else
                *n = '1';
        }
        else
            *n = toupper(*p);
    }

    /* process the extension if there is one */
    if (delim)
    {
        for (i = 0; i < 3 && *delim; i++, delim++, p++, n++)
            *n = toupper(*delim);
    }

    /* end of string */
    *n = 0;
}

