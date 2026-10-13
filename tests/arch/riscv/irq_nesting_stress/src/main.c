/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Stress nested interrupt handling with asynchronous sources.
 *
 * Three free-running timers at different interrupt levels fire with
 * mutually prime, jittered periods, so over time every instruction of the
 * trap entry/exit path of one interrupt gets preempted by another one.
 * Each ISR wakes a thread, so the reschedule and context switch paths at
 * ISR exit are exercised as well, while a low priority thread continuously
 * checks that its register state survives being preempted.
 *
 * Unlike tests/arch/common/interrupt (test_nested_isr), where the nested
 * interrupt is triggered from inside the outer ISR body, the preemption
 * point here is not controlled by software.
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/ztest.h>
#include <kernel_internal.h>
#include <hal/nrf_timer.h>

#define NUM_SOURCES 3
#define TIMER_FREQ_HZ 1000000

#define SRC_NODE(i) DT_ALIAS(stress_timer##i)

#define WORKER_STACK_SIZE 1024
#define VICTIM_STACK_SIZE 1024

struct source {
	NRF_TIMER_Type *timer;
	uint32_t irq;
	uint32_t period;
	uint32_t lfsr;
	struct k_sem sem;

	volatile uint32_t isr_count;
	volatile uint32_t preempted_count;
	volatile uint32_t wakeups;
	volatile uint32_t worker_errors;
};

static struct source sources[NUM_SOURCES] = {
	{
		.timer = (NRF_TIMER_Type *)DT_REG_ADDR(SRC_NODE(0)),
		.irq = DT_IRQN(SRC_NODE(0)),
		.period = CONFIG_TEST_PERIOD0_US,
		.lfsr = 0xACE1u,
	},
	{
		.timer = (NRF_TIMER_Type *)DT_REG_ADDR(SRC_NODE(1)),
		.irq = DT_IRQN(SRC_NODE(1)),
		.period = CONFIG_TEST_PERIOD1_US,
		.lfsr = 0xBEEFu,
	},
	{
		.timer = (NRF_TIMER_Type *)DT_REG_ADDR(SRC_NODE(2)),
		.irq = DT_IRQN(SRC_NODE(2)),
		.period = CONFIG_TEST_PERIOD2_US,
		.lfsr = 0x1D2Cu,
	},
};

static volatile bool stop;

static volatile uint32_t isr_errors;
static volatile uint32_t first_bad_nested;
static volatile uintptr_t first_bad_sp;
static volatile uint32_t max_nested;

static volatile uint32_t victim_iterations;
static volatile uint32_t victim_errors;

/* Threads must always run at CLIC interrupt level 0 (mintstatus.MIL) */
static volatile uint32_t thread_level_errors;
static volatile uint32_t first_bad_mintstatus;

static inline void check_thread_level(void)
{
	uint32_t mintstatus;

	__asm__ volatile("csrr %0, 0x346" : "=r"(mintstatus));
	if ((mintstatus >> 24) != 0u) {
		if (thread_level_errors++ == 0u) {
			first_bad_mintstatus = mintstatus;
			printk("thread %p running with mintstatus 0x%08x\n", k_current_get(),
			       mintstatus);
		}
	}
}

K_THREAD_STACK_ARRAY_DEFINE(worker_stacks, NUM_SOURCES, WORKER_STACK_SIZE);
static struct k_thread worker_threads[NUM_SOURCES];

K_THREAD_STACK_DEFINE(victim_stack, VICTIM_STACK_SIZE);
static struct k_thread victim_thread;

static inline uint32_t lfsr_next(uint32_t *state)
{
	uint32_t lsb = *state & 1u;

	*state >>= 1;
	if (lsb != 0u) {
		*state ^= 0xB400u;
	}

	return *state;
}

static void stress_isr(const void *arg)
{
	struct source *src = (struct source *)arg;
	uintptr_t irq_stack_lo = (uintptr_t)z_interrupt_stacks[0];
	uintptr_t irq_stack_hi = irq_stack_lo + sizeof(z_interrupt_stacks[0]);
	uint32_t nested = arch_curr_cpu()->nested;
	uintptr_t sp;
	uint32_t rnd;

	__asm__ volatile("mv %0, sp" : "=r"(sp));

	nrf_timer_event_clear(src->timer, NRF_TIMER_EVENT_COMPARE0);

	if ((nested == 0u) || (sp < irq_stack_lo) || (sp >= irq_stack_hi)) {
		if (isr_errors++ == 0u) {
			first_bad_nested = nested;
			first_bad_sp = sp;
		}
	}

	if (nested > 1u) {
		src->preempted_count++;
	}
	if (nested > max_nested) {
		max_nested = nested;
	}

	src->isr_count++;

	rnd = lfsr_next(&src->lfsr);
	nrf_timer_cc_set(src->timer, NRF_TIMER_CC_CHANNEL0, src->period + (rnd & 0x7u));

	for (volatile uint32_t i = 0; i < (rnd % CONFIG_TEST_ISR_MAX_SPIN); i++) {
	}

	if (IS_ENABLED(CONFIG_TEST_WAKE_THREADS)) {
		k_sem_give(&src->sem);
	}
}

static void worker(void *p1, void *p2, void *p3)
{
	struct source *src = p1;
	uint32_t n = 0;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (!stop) {
		/* tag must survive the context switches inside k_sem_take() */
		uint32_t tag = n * 0x9E3779B9u;

		__asm__ volatile("" : "+r"(tag));
		k_sem_take(&src->sem, K_FOREVER);
		__asm__ volatile("" : "+r"(tag));

		if (tag != n * 0x9E3779B9u) {
			src->worker_errors++;
		}
		check_thread_level();
		src->wakeups++;
		n++;
	}
}

static void victim(void *p1, void *p2, void *p3)
{
	uint32_t x = 0x12345678u;
	uint32_t y = ~x;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (!stop) {
		x = x * 1103515245u + 12345u;
		y = ~x;
		/* Hide the relation from the compiler so both stay live in registers */
		__asm__ volatile("" : "+r"(x), "+r"(y));
		if (y != ~x) {
			victim_errors++;
		}
		check_thread_level();
		victim_iterations++;
	}
}

static void timer_start(struct source *src)
{
	NRF_TIMER_Type *t = src->timer;

	nrf_timer_task_trigger(t, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(t, NRF_TIMER_TASK_CLEAR);
	nrf_timer_mode_set(t, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(t, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_prescaler_set(t, NRF_TIMER_PRESCALER_CALCULATE(NRF_TIMER_BASE_FREQUENCY_GET(t),
								  TIMER_FREQ_HZ));
	nrf_timer_cc_set(t, NRF_TIMER_CC_CHANNEL0, src->period);
	nrf_timer_shorts_enable(t, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
	nrf_timer_event_clear(t, NRF_TIMER_EVENT_COMPARE0);
	nrf_timer_int_enable(t, NRF_TIMER_INT_COMPARE0_MASK);
	irq_enable(src->irq);
	nrf_timer_task_trigger(t, NRF_TIMER_TASK_START);
}

static void timer_stop(struct source *src)
{
	NRF_TIMER_Type *t = src->timer;

	irq_disable(src->irq);
	nrf_timer_task_trigger(t, NRF_TIMER_TASK_STOP);
	nrf_timer_int_disable(t, NRF_TIMER_INT_COMPARE0_MASK);
	nrf_timer_event_clear(t, NRF_TIMER_EVENT_COMPARE0);
}

ZTEST(riscv_irq_nesting_stress, test_async_nested_irqs)
{
	/* VPR CLIC levels: higher value preempts lower */
	IRQ_CONNECT(DT_IRQN(SRC_NODE(0)), 1, stress_isr, &sources[0], 0);
	IRQ_CONNECT(DT_IRQN(SRC_NODE(1)), COND_CODE_1(CONFIG_TEST_SAME_PRIORITY, (1), (2)),
		    stress_isr, &sources[1], 0);
	IRQ_CONNECT(DT_IRQN(SRC_NODE(2)), COND_CODE_1(CONFIG_TEST_SAME_PRIORITY, (1), (3)),
		    stress_isr, &sources[2], 0);

	stop = false;

	for (int i = 0; i < NUM_SOURCES; i++) {
		k_sem_init(&sources[i].sem, 0, 1);
		k_thread_create(&worker_threads[i], worker_stacks[i],
				K_THREAD_STACK_SIZEOF(worker_stacks[i]), worker, &sources[i], NULL,
				NULL, K_PRIO_PREEMPT(1 + i), 0, K_NO_WAIT);
	}

	k_thread_create(&victim_thread, victim_stack, K_THREAD_STACK_SIZEOF(victim_stack), victim,
			NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);

	for (int i = 0; i < CONFIG_TEST_NUM_SOURCES; i++) {
		timer_start(&sources[i]);
	}

	k_msleep(CONFIG_TEST_DURATION_MS);

	for (int i = 0; i < CONFIG_TEST_NUM_SOURCES; i++) {
		timer_stop(&sources[i]);
	}

	stop = true;
	for (int i = 0; i < NUM_SOURCES; i++) {
		k_sem_give(&sources[i].sem);
		k_thread_join(&worker_threads[i], K_FOREVER);
	}
	k_thread_join(&victim_thread, K_FOREVER);

	for (int i = 0; i < NUM_SOURCES; i++) {
		TC_PRINT("source %d (irq %u): isr %u, preempted another %u, wakeups %u\n", i,
			 sources[i].irq, sources[i].isr_count, sources[i].preempted_count,
			 sources[i].wakeups);
	}
	TC_PRINT("max nesting depth %u, victim iterations %u\n", max_nested, victim_iterations);
	TC_PRINT("threads seen at non-zero interrupt level %u times (first mintstatus 0x%08x)\n",
		 thread_level_errors, first_bad_mintstatus);

	zassert_equal(isr_errors, 0,
		      "%u ISRs ran outside the IRQ stack or with nested == 0 (first: nested %u, "
		      "sp 0x%lx)",
		      isr_errors, first_bad_nested, (unsigned long)first_bad_sp);
	zassert_equal(victim_errors, 0, "victim thread register state corrupted %u times",
		      victim_errors);
	zassert_equal(thread_level_errors, 0,
		      "threads ran at a non-zero CLIC interrupt level %u times (mintstatus 0x%08x)",
		      thread_level_errors, first_bad_mintstatus);

	for (int i = 0; i < NUM_SOURCES; i++) {
		zassert_equal(sources[i].worker_errors, 0,
			      "worker %d register state corrupted %u times", i,
			      sources[i].worker_errors);
	}

	for (int i = 0; i < CONFIG_TEST_NUM_SOURCES; i++) {
		zassert_true(sources[i].isr_count > 0, "source %d never fired", i);
	}

	if ((CONFIG_TEST_NUM_SOURCES > 1) && !IS_ENABLED(CONFIG_TEST_SAME_PRIORITY)) {
		zassert_true(sources[1].preempted_count + sources[2].preempted_count > 0,
			     "no nesting observed, tune CONFIG_TEST_PERIOD*_US / "
			     "CONFIG_TEST_ISR_MAX_SPIN");
	}
}

ZTEST_SUITE(riscv_irq_nesting_stress, NULL, NULL, NULL, NULL, NULL);
