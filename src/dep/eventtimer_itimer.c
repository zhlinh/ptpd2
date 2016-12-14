/**
 * EventTimer基于interval timers的具体实现
 */

/*-
 * Copyright (c) 2012-2015 Wojciech Owczarek,
 * Copyright (c) 2011-2012 George V. Neville-Neil,
 *                         Steven Kreuzer, 
 *                         Martin Burnicki, 
 *                         Jan Breuer,
 *                         Gael Mace, 
 *                         Alexandre Van Kempen,
 *                         Inaqui Delgado,
 *                         Rick Ratzel,
 *                         National Instruments.
 * Copyright (c) 2009-2010 George V. Neville-Neil, 
 *                         Steven Kreuzer, 
 *                         Martin Burnicki, 
 *                         Jan Breuer,
 *                         Gael Mace, 
 *                         Alexandre Van Kempen
 *
 * Copyright (c) 2005-2008 Kendall Correll, Aidan Williams
 *
 * All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   eventtimer_itimer.c
 * @date   Wed Oct 1 00:41:26 2014
 * 
 * @brief  EventTimer implementation using interval timers
 * 
 * The original interval timer timer implementation.
 */

#include "../ptpd.h"

#define US_TIMER_INTERVAL (62500)

static volatile unsigned int elapsed;

static void eventTimerStart_itimer(EventTimer *timer, double interval);
static void eventTimerStop_itimer(EventTimer *timer);
static void eventTimerReset_itimer(EventTimer *timer);
static void eventTimerShutdown_itimer(EventTimer *timer);
static Boolean eventTimerIsRunning_itimer(EventTimer *timer);
static Boolean eventTimerIsExpired_itimer(EventTimer *timer);

static void itimerUpdate(EventTimer *et);
static void timerSignalHandler(int sig);

void
setupEventTimer(EventTimer *timer)
{

	if(timer == NULL) {
	    return;
	}

	memset(timer, 0, sizeof(EventTimer));

	timer->start = eventTimerStart_itimer;
	timer->stop = eventTimerStop_itimer;
	timer->reset = eventTimerReset_itimer;
	timer->shutdown = eventTimerShutdown_itimer;
	timer->isExpired = eventTimerIsExpired_itimer;
	timer->isRunning = eventTimerIsRunning_itimer;
}

static void
eventTimerStart_itimer(EventTimer *timer, double interval)
{

	timer->expired = FALSE;
	timer->running = TRUE;

	/*
	 *  US_TIMER_INTERVAL defines the minimum interval between sigalarms.
	 *  timerStart has a float parameter for the interval, which is casted to integer.
	 *  very small amounts are forced to expire ASAP by setting the interval to 1
	 */
	/**
	 * 最小信号间隔为US_TIMER_INTERVAL(即(62500us))
	 * 本函数的参数传入了double的参数interval，除以US_TIMER_INTERVAL得到一个整数的定时器剩余时间timer->itimerLeft
	 * 即定时器剩余时间timer->itimerLeft的单位为最小信号间隔US_TIMER_INTERVAL
	 * 如果得到的定时器剩余时间timer->itimerLeft为0，则将其设为1
	 */

	timer->itimerLeft = (interval * 1E6) / US_TIMER_INTERVAL;
	if(timer->itimerLeft == 0){
		/*
		 * the interval is too small, raise it to 1 to make sure it expires ASAP
		 */ 
		timer->itimerLeft = 1;
	}

	/**
	 * 将定时器剩余时间timer->itimerLeft赋值给timer->itimerInterval
	 */
	timer->itimerInterval = timer->itimerLeft;

	DBG2("timerStart:     Set timer %s to %f  New interval: %d; new left: %d\n", timer->id, interval, timer->itimerLeft , timer->itimerInterval);
}

static void
eventTimerStop_itimer(EventTimer *timer)
{

	timer->itimerInterval = 0;
	timer->running = FALSE;
	DBG2("timerStop:      Stopping timer %s\n", timer->id);

}

/**
 * eventTimerIsRunning_itimer()和eventTimerIsExpired_itimer()函数
 * 都会调用itimerUpdate()
 */
static void
itimerUpdate(EventTimer *et)
{

	EventTimer *timer = NULL;

	/**
	 * elapsed为本文件内部全局静态unsigned int变量
	 * 时间没有真的流逝时(elapsed <= 0)直接返回
	 */
	if (elapsed <= 0)
		return;

	/*
	 * if time actually passed, then decrease every timer left
	 * the one(s) that went to zero or negative are:
	 *  a) rearmed at the original time (ignoring the time that may have passed ahead)
	 *  b) have their expiration latched until timerExpired() is called
	 */
	/**
	 * 如果时间真的流逝了(elapsed > 0)，就减少每个定时器的剩余时间
	 * 采用轮询的方式，如果定时器数量过多，则效率可能会比较低
	 * 定时器剩余时间小于等于0的情况有:
	 *  1) 更新原始时间(忽略了可能已经过去的时间)时
	 *  2) 它们的到期被锁定时，直到timerExpired()被调用才会被解锁
	 */

	for(timer = et->_first; timer != NULL; timer = timer->_next) {
	    if ( (timer->itimerInterval > 0) && ((timer->itimerLeft -= elapsed) <= 0)) {
			timer->itimerLeft = timer->itimerInterval;
			timer->expired = TRUE;
		DBG("TimerUpdate:    Timer %s has now expired.  Re-armed with interval %d\n", timer->id, timer->itimerInterval);
	    }
	}

	/**
	 * 将elapsed清零重新累加
	 */
	elapsed = 0;

}



static void
eventTimerReset_itimer(EventTimer *timer)
{
}

static void
eventTimerShutdown_itimer(EventTimer *timer)
{
}


static Boolean
eventTimerIsRunning_itimer(EventTimer *timer)
{

	itimerUpdate(timer);

	DBG2("timerIsRunning:   Timer %s %s running\n", timer->id,
		timer->running ? "is" : "is not");

	return timer->running;
}

static Boolean
eventTimerIsExpired_itimer(EventTimer *timer)
{

	Boolean ret;

	itimerUpdate(timer);

	ret = timer->expired;

	DBG2("timerIsExpired:   Timer %s %s expired\n", timer->id,
		timer->expired ? "is" : "is not");

	/**
	 * 如果expired则将其重新置为FALSE
	 */
	if(ret) {
	    timer->expired = FALSE;
	}

	return ret;

}

void 
startEventTimers(void)
{
	struct itimerval itimer;

	DBG("initTimer\n");

/**
 * SIG_IGN表示忽略该信号量
 */
#ifdef __sun
	sigset(SIGALRM, SIG_IGN);
#else
	signal(SIGALRM, SIG_IGN);
#endif /* __sun */

	elapsed = 0;
	/**
	 * 将下一次启动时间和间隔都设置为US_TIMER_INTERVAL
     */
	itimer.it_value.tv_sec = itimer.it_interval.tv_sec = 0;
	itimer.it_value.tv_usec = itimer.it_interval.tv_usec = 
	    US_TIMER_INTERVAL;

/**
 * 将SIGALRM的回调函数设置为timerSignalHandler
 */
#ifdef __sun
	sigset(SIGALRM, timerSignalHandler);
#else	
	signal(SIGALRM, timerSignalHandler);
#endif /* __sun */
    /**
     * 设置定时器，下一次启动时间和间隔都为US_TIMER_INTERVAL
     */
	setitimer(ITIMER_REAL, &itimer, 0);
}

void 
shutdownEventTimers(void)
{

/**
 * SIG_IGN表示忽略该信号量
 */
#ifdef __sun
	sigset(SIGALRM, SIG_IGN);
#else
	signal(SIGALRM, SIG_IGN);
#endif /* __sun */
}

static void 
timerSignalHandler(int sig)
{
	/**
	 * 定时器的回调函数，作用为增加elapsed
	 */
	elapsed++;
	/* be sure to NOT call DBG in asynchronous handlers! */
}

