void read_numbers_from_string (const char *str, void (*callback)(double))
{
    while (*str)
    {
        char *p = strchr(str, '=');
        if (!p) break;

        errno = 0;
        char *endp;
        double n = strtod(p + 1, &endp);
        if (endp > p + 1 && !errno)
            callback(n);

        str = endp;
    }
}