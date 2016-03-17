/*
 * Unit test suite for *scanf functions.
 *
 * Copyright 2002 Uwe Bonnes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA
 *
 * From winehq tests.
 */

#include <stdio.h>
#include <string.h>

#define str_(x) #x
#define str(x) str_(x)

#define ok(cond, ...)                           \
  do {                                          \
    total++;                                    \
    if (cond)                                   \
      {                                         \
        printk("passed:" str(__LINE__)"\n");        \
        ok++;                                   \
      }                                         \
    else                                        \
      printk("failed:" str(__LINE__) ":" __VA_ARGS__);   \
  } while (0)

int ok = 0;
int total = 0;

void app_start()
{
  char buffer[100], buffer1[100];
  char format[20];
  int result, ret;
  char c;
  void *ptr;
#ifdef CONFIG_LIBC_FORMATTER_FLOAT
  float res1, res2;
  double res11, res12;
#endif
  static const char pname[]=" St. Petersburg, Florida\n";
  int hour=21,min=59,sec=20;
  int  number,number_so_far;

  /* check EOF */
  strcpy(buffer,"");
  ret = sscanf(buffer, "%d", &result);
  ok( ret == EOF,"sscanf returns %x instead of %x\n", ret, EOF );

  /* check %p */
  ok( sscanf("000000000046F170", "%p", &ptr) == 1, "sscanf failed\n"  );
  ok( ptr == (void *)0x46F170,"sscanf reads %p instead of %x\n", ptr, 0x46F170 );

  ok( sscanf("0046F171", "%p", &ptr) == 1, "sscanf failed\n"  );
  ok( ptr == (void *)0x46F171,"sscanf reads %p instead of %x\n", ptr, 0x46F171 );

  ok( sscanf("046F171", "%p", &ptr) == 1, "sscanf failed\n"  );
  ok( ptr == (void *)0x46F171,"sscanf reads %p instead of %x\n", ptr, 0x46F171 );

  ok( sscanf("46F172", "%p", &ptr) == 1, "sscanf failed\n"  );
  ok( ptr == (void *)0x46F172,"sscanf reads %p instead of %x\n", ptr, 0x46F172 );

  ok( sscanf("0x46F173", "%p", &ptr) == 1, "sscanf failed\n"  );
  ok( ptr == NULL,"sscanf reads %p instead of %x\n", ptr, 0 );

#if 0
  ok( sscanf("-46F174", "%p", &ptr) == 1, "sscanf failed\n"  );
  ok( ptr == (void *)(uintptr_t)-0x46f174,"sscanf reads %p instead of %p\n",
      ptr, (void *)(uintptr_t)-0x46f174 );

  ok( sscanf("+46F175", "%p", &ptr) == 1, "sscanf failed\n"  );
  ok( ptr == (void *)0x46F175,"sscanf reads %p instead of %x\n", ptr, 0x46F175 );
#endif

  /* check %p with no hex digits */
  ok( sscanf("1233", "%p", &ptr) == 1, "sscanf failed\n"  );
  ok( ptr == (void *)0x1233,"sscanf reads %p instead of %x\n", ptr, 0x1233 );

  ok( sscanf("1234", "%P", &ptr) == 1, "sscanf failed\n"  );
  ok( ptr == (void *)0x1234,"sscanf reads %p instead of %x\n", ptr, 0x1234 );

  /* check %x */
  strcpy(buffer,"0x519");
  ok( sscanf(buffer, "%x", &result) == 1, "sscanf failed\n"  );
  ok( result == 0x519,"sscanf reads %x instead of %x\n", result, 0x519 );

  strcpy(buffer,"0x51a");
  ok( sscanf(buffer, "%x", &result) == 1, "sscanf failed\n" );
  ok( result == 0x51a ,"sscanf reads %x instead of %x\n", result, 0x51a );

  strcpy(buffer,"0519");
  ok( sscanf(buffer, "%x", &result) == 1, "sscanf failed\n"  );
  ok( result == 0x519,"sscanf reads %x instead of %x\n", result, 0x519 );

  strcpy(buffer,"51a");
  ok( sscanf(buffer, "%x", &result) == 1, "sscanf failed\n" );
  ok( result == 0x51a ,"sscanf reads %x instead of %x\n", result, 0x51a );

  strcpy(buffer,"0x51g");
  ok( sscanf(buffer, "%x", &result) == 1, "sscanf failed\n" );
  ok( result == 0x51, "sscanf reads %x instead of %x\n", result, 0x51 );

  result = 0;
  ret = sscanf("-1", "%x", &result);
  ok(ret == 1, "Wrong number of arguments read: %d (expected 1)\n", ret);
  ok(result == -1, "Read %d, expected -1\n", result);

  result = 0;
  ret = sscanf("-1647", "%x", &result);
  ok(ret == 1, "Wrong number of arguments read: %d (expected 1)\n", ret);
  ok(result == -0x1647, "Read %d, expected -1647\n", result);

  /* check % followed by any char */
  strcpy(buffer,"\"%12@");
  strcpy(format,"%\"%%%d%@");  /* work around gcc format check */
  ok( sscanf(buffer, format, &result) == 1, "sscanf failed\n" );
  ok( result == 12, "sscanf reads %x instead of %x\n", result, 12 );

#ifdef CONFIG_LIBC_FORMATTER_FLOAT

  /* Check double */
  strcpy(buffer,"1234.5678 32.5e18");
  ret = sscanf(buffer,"%lf %lf",&res11, &res12);
  ok( ret == 2, "expected 2, got %u\n", ret);
  ok( (res11 > 1234.5677f) && (res11 < 1234.5679f) &&
      (res12 > 32.4e18) && (res12 < 32.6e18),
      "Error reading floats %lf %lf\n", res11, res12);

  /* Check float */
  strcpy(buffer,".42 -444.33e-2");
  ret = sscanf(buffer,"%f %f",&res1, &res2);
  ok( ret == 2, "expected 2, got %u\n", ret);
  ok( (res1 > .41f) && (res1 < .43f) &&
      (res2 < -4.4432f) && (res2 > -4.4434f),
      "Error reading floats %f %f\n", res1, res2);
#endif

  /* check strings */
  ret = sprintf(buffer," %s", pname);
  ok( ret == 26, "expected 26, got %u\n", ret);
  ret = sscanf(buffer,"%*c%[^\n]",buffer1);
  ok( ret == 1, "Error with format \"%s\"\n","%*c%[^\n]");
  ok( strncmp(pname,buffer1,strlen(buffer1)) == 0, "Error with \"%s\" \"%s\"\n",pname, buffer1);

  ret = sscanf("abcefgdh","%*[a-cg-e]%c",&buffer[0]);
  ok( ret == 1, "Error with format \"%s\"\n","%*[a-cg-e]%c");
  ok( buffer[0] == 'd', "Error with \"abcefgdh\" \"%c\"\n", buffer[0]);

  ret = sscanf("abcefgdh","%*[a-cd-dg-e]%c",&buffer[0]);
  ok( ret == 1, "Error with format \"%s\"\n","%*[a-cd-dg-e]%c");
  ok( buffer[0] == 'h', "Error with \"abcefgdh\" \"%c\"\n", buffer[0]);

  /* check digits */
  ret = sprintf(buffer,"%d:%d:%d",hour,min,sec);
  ok( ret == 8, "expected 8, got %u\n", ret);
  ret = sscanf(buffer,"%d%n",&number,&number_so_far);
  ok(ret == 1 , "problem with format arg \"%%d%%n\", ret=%i\n", ret);
  ok(number == hour,"Read wrong arg %d instead of %d\n",number, hour);
  ok(number_so_far == 2,"Read wrong arg for \"%%n\" %d instead of 2\n",number_so_far);

  ret = sscanf(buffer+2,"%*c%n",&number_so_far);
  ok(ret == 0 , "problem with format arg \"%%*c%%n\"\n");
  ok(number_so_far == 1,"Read wrong arg for \"%%n\" %d instead of 2\n",number_so_far);

  /* Check %i according */
  strcpy(buffer,"123");
  ret = sscanf(buffer, "%i", &result);
  ok(ret == 1, "Wrong number of arguments read: %d\n", ret);
  ok(result == 123, "Wrong number read\n");
  result = 0;
  ret = sscanf("-1", "%i", &result);
  ok(ret == 1, "Wrong number of arguments read: %d (expected 1)\n", ret);
  ok(result == -1, "Read %d, expected -1\n", result);
  ret = sscanf(buffer, "%d", &result);
  ok(ret == 1, "Wrong number of arguments read: %d\n", ret);
  ok(result == 123, "Wrong number read\n");
  result = 0;
  ret = sscanf("-1", "%d", &result);
  ok(ret == 1, "Wrong number of arguments read: %d (expected 1)\n", ret);
  ok(result == -1, "Read %d, expected -1\n", result);

  /* Check %i for octal and hexadecimal input */
  result = 0;
  strcpy(buffer,"017");
  ret = sscanf(buffer, "%i", &result);
  ok(ret == 1, "Wrong number of arguments read: %d\n", ret);
  ok(result == 15, "Wrong number read\n");
  result = 0;
  strcpy(buffer,"0x17");
  ret = sscanf(buffer, "%i", &result);
  ok(ret == 1, "Wrong number of arguments read: %d\n", ret);
  ok(result == 23, "Wrong number read\n");

  /* %o */
  result = 0;
  ret = sscanf("-1", "%o", &result);
  ok(ret == 1, "Wrong number of arguments read: %d (expected 1)\n", ret);
  ok(result == -1, "Read %d, expected -1\n", result);

  /* %u */
  result = 0;
  ret = sscanf("-1", "%u", &result);
  ok(ret == 1, "Wrong number of arguments read: %d (expected 1)\n", ret);
  ok(result == -1, "Read %d, expected -1\n", result);

  /* Check %c */
  strcpy(buffer,"a");
  c = 0x55;
  ret = sscanf(buffer, "%c", &c);
  ok(ret == 1, "Wrong number of arguments read: %d\n", ret);
  ok(c == 'a', "Field incorrect: '%c'\n", c);

  strcpy(buffer," a");
  c = 0x55;
  ret = sscanf(buffer, "%c", &c);
  ok(ret == 1, "Wrong number of arguments read: %d\n", ret);
  ok(c == ' ', "Field incorrect: '%c'\n", c);

  strcpy(buffer,"18:59");
  c = 0x55;
  ret = sscanf(buffer, "%d:%d%c", &hour, &min, &c);
  ok(ret == 2, "Wrong number of arguments read: %d\n", ret);
  ok(hour == 18, "Field 1 incorrect: %d\n", hour);
  ok(min == 59, "Field 2 incorrect: %d\n", min);
  ok(c == 0x55, "Field 3 incorrect: 0x%02x\n", c);

  /* Check %n (also whitespace in format strings and %s) */
  buffer[0]=0; buffer1[0]=0;
  ret = sscanf("abc   def", "%s %n%s", buffer, &number_so_far, buffer1);
  ok(strcmp(buffer, "abc")==0, "First %%s read incorrectly: %s\n", buffer);
  ok(strcmp(buffer1,"def")==0, "Second %%s read incorrectly: %s\n", buffer1);
  ok(number_so_far==6, "%%n yielded wrong result: %d\n", number_so_far);
  ok(ret == 2, "%%n shouldn't count as a conversion: %d\n", ret);

  /* Check where %n matches to EOF in buffer */
  strcpy(buffer, "3:45");
  ret = sscanf(buffer, "%d:%d%n", &hour, &min, &number_so_far);
  ok(ret == 2, "Wrong number of arguments read: %d\n", ret);
  ok(number_so_far == 4, "%%n yielded wrong result: %d\n", number_so_far);

  strcpy(buffer, "8998");
  ret = sscanf(buffer, "%2hhd%2hhd", buffer1 + 1, buffer1);
  ok(ret == 2, "Wrong number of arguments read: %d\n", ret);
  ok(buffer1[1] == 89, "%%n yielded wrong result\n");
  ok(buffer1[0] == 98, "%%n yielded wrong result\n");

  assert(total > 70);
  printk("++RESULT++%i++ (%i passed / %i total)\n", total-ok, ok, total);
}

