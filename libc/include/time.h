
struct timeval;
struct timezone;

static inline int gettimeofday(struct timeval *tv, struct timezone *tz)
{
  return 0;
}

static inline int settimeofday(const struct timeval *tv, const struct timezone *tz)
{
  return 0;
}

static inline int time(time_t *t)
{
  return 0;
}

