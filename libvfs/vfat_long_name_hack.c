
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
 *  ! This function processes directly the 
 *  string argument.
 */
void touppershortname(char* path)
{
    /* sanity check */
    assert(path != NULL);
    if (*path == 0) return;

    char *p = path;
    char *delim;
    delim = strchr(path, '.');

    size_t i;

    /* process the body name */
    for (i = 0; i < 8 && *p && (p != delim); i++, p++)
    {
        if ((delim - path) > 8 && i >= 6)
        {
            if (i == 6)
                *p = '~';
            else
                *p = '1';
        }
        else
            *p = toupper(*p);
    }

    /* process the extension if there is one */
    if (delim)
    {
        for (i = 0; i < 3 && *delim; i++, delim++, p++)
            *p = toupper(*delim);
    }

    /* end of string */
    *p = 0;
}

