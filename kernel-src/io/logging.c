#include <mutex.h>

MUTEX_DEFINE(printf_mutex);

static void nohook(char c) {

}

static void (*hook)(char) = nohook;

// hook used by mpaland printf
void arch_early_log(char c);
void _putchar(char c) {
	// hook(c);

	// if (hook != arch_early_log)
	arch_early_log(c);
}

void logging_sethook(void (*fun)(char)) {
	hook = fun;
}
