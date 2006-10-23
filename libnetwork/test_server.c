#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <netinet/if.h>
#include <netinet/in.h>
#include <netinet/libudp.h>

#include <stdio.h>
#include <string.h>

# define ERR_SCAN	1
# define ERR_PARSE	2
# define ERR_MATH	3
# define ERR_UNK	4

# define BLOCK_SIZE	1024
# define ERROR		(-1)
# define EOF		0
# define NUMBER		1
# define PLUS		2
# define MINUS		3
# define TIMES		4
# define DIV		5
# define MOD		6
# define LPAREN		7
# define RPAREN		8

# define ORDER_PRE	1
# define ORDER_MID	2
# define ORDER_POST	3


struct	s_opt
{
  char		*base;
  int_fast32_t	base_len;
  char		*ops;
  int_fast32_t	order;
};


struct		s_parsing
{
  char	*line;
  int_fast32_t	pos;
  int_fast32_t	result;
  int_fast32_t	paren;
  struct s_opt	opt;
  uint_fast8_t	error;
};

static void	scan_for_op(struct s_opt *opt, char *line, int_fast32_t *pos, int_fast32_t *tok)
{
  if (line[*pos] == opt->ops[0])
    *tok = PLUS;
  else
    if (line[*pos] == opt->ops[1])
      *tok = MINUS;
    else
      if (line[*pos] == opt->ops[2])
	*tok = TIMES;
      else
	if (line[*pos] == opt->ops[3])
	  *tok = DIV;
	else
	  if (line[*pos] == opt->ops[4])
	    *tok = MOD;
	  else
	    if (line[*pos] == opt->ops[5])
	      *tok = LPAREN;
	    else
	      if (line[*pos] == opt->ops[6])
		*tok = RPAREN;
  if (*tok != ERROR)
    ++(*pos);
}

static int_fast32_t	is_in_base(char c, const char *base)
{
  int_fast32_t		idx = 0;

  while (*base)
    {
      if (*base == c)
	return idx;
      ++idx;
      ++base;
    }
  return -1;
}

static int_fast32_t	scan_for_number(struct s_opt	*opt,
				char			*line,
				int_fast32_t		*pos,
				int_fast32_t		*tok)
{
  int_fast32_t		num = 0;
  int_fast32_t		idx;

  if ((idx = is_in_base(line[*pos], opt->base)) == -1)
    return 0;
  do
    {
      num = num * opt->base_len + idx;
      ++(*pos);
    }
  while ((idx = is_in_base(line[*pos], opt->base)) != -1);
  *tok = NUMBER;
  return num;
}

static int_fast32_t	lex(struct s_parsing *par, struct s_opt *opt, int_fast32_t *num, char **line, int_fast32_t *pos)
{
  int_fast32_t	res = ERROR;

  if (!(*line)[*pos])
    return EOF;
  scan_for_op(opt, *line, pos, &res);
  if (res == ERROR)
    *num = scan_for_number(opt, *line, pos, &res);
  if (res == ERROR)
    par->error = ERR_SCAN;
  return res;
}

static void	rule_A(struct s_parsing *par);

static void	do_compute(struct s_parsing *par, int_fast32_t acc, int_fast32_t tok)
{
  if (tok == PLUS)
    par->result = acc + par->result;
  else
    if (tok == MINUS)
      par->result = acc - par->result;
    else
      if (tok == TIMES)
	par->result = acc * par->result;
      else
	if (tok == DIV)
	  {
	    if (par->result == 0)
	      par->error = ERR_MATH;
	    else
	      par->result = acc / par->result;
	  }
	else
	  if (tok == MOD)
	    {
	      if (par->result == 0)
		par->error = ERR_MATH;
	      else
		par->result = acc % par->result;
	    }
}

static void	rule_E(struct s_parsing *par)
{
  int_fast32_t		num;
  int_fast32_t		tok;

  tok = lex(par, &par->opt, &num, &par->line, &par->pos);
  if (tok == NUMBER)
    par->result = num;
  else
    par->error = ERR_PARSE;
}

static void	rule_D(struct s_parsing *par)
{
  int_fast32_t		num;
  int_fast32_t		tok;
  int_fast32_t		bt;

  bt = par->pos;
  tok = lex(par, &par->opt, &num, &par->line, &par->pos);
  if (tok == LPAREN)
    {
      ++par->paren;
      rule_A(par);
    }
  else
    {
      par->pos = bt;
      rule_E(par);
    }
}

static void	rule_C(struct s_parsing *par)
{
  int_fast32_t		num;
  int_fast32_t		tok;
  int_fast32_t		bt;
  int_fast32_t		sign = 0;

  while (par->error == 0)
    {
      bt = par->pos;
      tok = lex(par, &par->opt, &num, &par->line, &par->pos);
      if (tok == PLUS || tok == MINUS)
	{
	  if (tok == MINUS)
	    sign = (sign + 1) % 2;
	}
      else
	{
	  par->pos = bt;
	  rule_D(par);
	  if (sign)
	    do_compute(par, 0, MINUS);
	  break;
	}
    }
}

static void	rule_B(struct s_parsing *par)
{
  int_fast32_t		num;
  int_fast32_t		compute;
  int_fast32_t		tok;
  int_fast32_t		bt;

  rule_C(par);
  while (par->error == 0)
    {
      bt = par->pos;
      tok = lex(par, &par->opt, &num, &par->line, &par->pos);
      if (tok == TIMES || tok == DIV || tok == MOD)
	{
	  compute = par->result;
	  rule_C(par);
	  do_compute(par, compute, tok);
	}
      else
	{
	  par->pos = bt;
	  break;
	}
    }
}

static void	rule_A(struct s_parsing *par)
{
  int_fast32_t		compute;
  int_fast32_t		tok;

  rule_B(par);
  while (par->error == 0)
    {
      tok = lex(par, &par->opt, &compute, &par->line, &par->pos);
      if (tok == PLUS || tok == MINUS)
	{
	  compute = par->result;
	  rule_B(par);
	  do_compute(par, compute, tok);
	}
      else
	if (tok == RPAREN)
	  {
	    if (--par->paren < 0)
	      par->error = ERR_PARSE;
	    break;
	  }
	else
	  if (tok == EOF || tok == RPAREN)
	    break;
	  else
	    par->error = ERR_PARSE;
    }
}

void			parse(char *line, char *result)
{
  struct s_parsing	par;

  par.opt.base = "0123456789";
  par.opt.base_len = 10;
  par.opt.ops = "+-*/%()";
  par.opt.order = ORDER_MID;

  par.result = 0;
  par.line = line;
  par.pos = 0;
  par.paren = 0;
  par.error = 0;
  rule_A(&par);
  if (par.paren || par.error == ERR_PARSE)
    strcpy(result, "Parse error");
  else if (par.error == ERR_SCAN)
    strcpy(result, "Scan error");
  else if (par.error == ERR_MATH)
    strcpy(result, "Arithmetic error");
  else
    sprintf(result, "%d", par.result);
}


UDP_CALLBACK(test_add)
{
  char		result[20];
  char		*op;

  op = mem_alloc(size + 1, MEM_SCOPE_SYS);
  memcpy(op, data, size);
  op[size] = 0;

  parse(op, result);

  mem_free(op);

  local->port = 0;
  remote->port = htons(4242);
  udp_send(local, remote, result, strlen(result));
}

/*
 * test main.
 */

void			eval_server()
{
  struct net_udp_addr_s	listen;
  error_t		err;

  IPV4_ADDR_SET(listen.address, htonl(INADDR_ANY));
  listen.port = htons(4242);

  err = udp_callback(&listen, test_add, NULL);
  if (err)
    printf("err = %d\n", err);

  printf("Da ult1m4t3 H4xX0r 3v4L3xPr 53rv3r f0R MuT3k l15t3N1nG 0n :4242\n");
}

