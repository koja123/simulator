/*********************************************************************
 *
 * AUTHORIZATION TO USE AND DISTRIBUTE
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that: 
 *
 * (1) source code distributions retain this paragraph in its entirety, 
 *  
 * (2) distributions including binary code include this paragraph in
 *     its entirety in the documentation or other materials provided 
 *     with the distribution, and 
 *
 * (3) all advertising materials mentioning features or use of this 
 *     software display the following acknowledgment:
 * 
 *      "This product includes software written and developed 
 *       by Code 5520 of the Naval Research Laboratory (NRL)." 
 *         
 *  The name of NRL, the name(s) of NRL  employee(s), or any entity
 *  of the United States Government may not be used to endorse or
 *  promote  products derived from this software, nor does the 
 *  inclusion of the NRL written and developed software  directly or
 *  indirectly suggest NRL or United States  Government endorsement
 *  of this product.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 ********************************************************************/

#include "protoTimer.h"
#include "protoDebug.h"

#include <stdio.h>  // for getchar() debug

namespace NrlProtolibPort //ScenSim-Port://
{

/*!
A generic timer class which will notify a ProtoTimer::Listener upon timeout.
*/

ProtoTimer::ProtoTimer()
 : listener(NULL), interval(1.0), repeat(0), repeat_count(0),
   timeout(0),mgr(NULL), prev(NULL), next(NULL)
{

}

ProtoTimer::~ProtoTimer()
{
    if (IsActive()) Deactivate();
    if (listener)
    {
        delete listener;
        listener = NULL;
    }
}




bool ProtoTimer::Reschedule()
{
    if (IsActive())
    {
        ProtoTimerMgr* theMgr = mgr;
        bool updatePending = theMgr->update_pending;
        theMgr->update_pending = true;
        theMgr->DeactivateTimer(*this);
        theMgr->update_pending = updatePending;
        theMgr->ActivateTimer(*this);
        return true;
    }
    else
    {
        DMSG(0, "ProtoTimer::Reschedule() error: timer not active\n");
        return false;
    }
}  // end ProtoTimer::Reschedule()

// This method stretches (factor > 1.0) or
// compresses the timer interval, rescheduling
// the timer if it is active
// (note the repeat count is not impacted)

void ProtoTimer::Scale(double factor)
{
    if (IsActive())
    {
        // Calculate, reschedule and then adjust interval
        double newInterval = factor*interval;
        double timeRemaining = GetTimeRemaining();
        if (timeRemaining > 0.0)
        {
            interval = factor*timeRemaining;
            int repeatCountSaved = repeat_count;
            Reschedule();
            repeat_count = repeatCountSaved;
        }
        interval = newInterval;
    }
    else
    {
        interval *= factor;
    }
}  // end ProtoTimer::Scale()

void ProtoTimer::Deactivate()
{
    ASSERT(IsActive());
    mgr->DeactivateTimer(*this);
}

double ProtoTimer::GetTimeRemaining() const
{
    if (NULL != mgr)
    {
        double timeRemaining = is_precise ?
                                  ProtoTimerMgr::PrecisionTimeDelta(timeout, mgr->GetPrecisionTime()) :
                                  (timeout - mgr->GetPulseTime());
        if (timeRemaining < 0.0) timeRemaining = 0.0;
        return timeRemaining;
    }
    else
    {
        return -1.0;
    }
}  // end ProtoTimer::GetTimeRemaining()

#ifdef WIN32
bool proto_performance_counter_init = false;
LARGE_INTEGER proto_performance_counter_frequency = {0, 0};
#ifdef _WIN32_WCE
long proto_performance_counter_offset = 0;
long proto_system_time_last_sec = 0;
unsigned long proto_system_count_roll_sec = 0;
LARGE_INTEGER proto_system_count_last = {0, 0};
#endif // _WIN32_WCE
#endif // WIN32

/*!  This class manages ProtoTimer instances when they are
"activated". The ProtoDispatcher derives from this to manage
ProtoTimers for an application.  (The ProtoSimAgent base class
contains a ProtoTimerMgr to similarly manage timers for a simulation
instance).
*/

ProtoTimerMgr::ProtoTimerMgr()
: update_pending(false), scheduled_timeout(-1.0),
  long_head(NULL), long_tail(NULL), short_head(NULL), short_tail(NULL)
{
    pulse_timer.SetListener(this, &ProtoTimerMgr::OnPulseTimeout);
    pulse_timer.SetInterval(1.0);
    pulse_timer.SetRepeat(-1);
}

ProtoTimerMgr::~ProtoTimerMgr()
{
    // (TBD) Uninstall or halt, deactivate all timers ...
}

void ProtoTimerMgr::GetSystemTime(struct timeval& currentTime)
{
    ::ProtoSystemTime(currentTime);
}  // end ProtoTimerMgr::GetSystemTime()

const double ProtoTimerMgr::PRECISION_TIME_THRESHOLD = 4.0;
const double ProtoTimerMgr::PRECISION_TIME_MODULUS = 16.0;
const double ProtoTimerMgr::PRECISION_TIME_HALF = 8.0;
const long ProtoTimerMgr::PRECISION_TIME_MASK = 0x0000000f;

void ProtoTimerMgr::OnSystemTimeout()
{
    scheduled_timeout = -1.0;
    update_pending = true;
    ProtoTimer* next = short_head;
    double now = GetPrecisionTime();
    while (next)
    {
        // This works within a microsecond of accuracy
        if (PrecisionTimeDelta(next->timeout, now) < (double)1.0e-06)
        {
            if(next->DoTimeout())
            {
                if (next->IsActive())
                {
                    RemoveShortTimer(next);
                    int repeatCount = next->repeat_count;
                    if (repeatCount)
                    {
                        ReactivateTimer(next, now);
                        if (repeatCount > 0) repeatCount--;
                        next->repeat_count = repeatCount;
                    }
                }
            }
            next = short_head;
        }
        else
        {
            next = NULL;
        }
    }
    update_pending = false;
    Update();
}  // ProtoTimerMgr::OnSystemTimeout()

bool ProtoTimerMgr::OnPulseTimeout(ProtoTimer& /*theTimer*/)
{
    ProtoTimer* next = long_head;
    pulse_mark += 1.0;
    double pulseCurrent = pulse_mark;
    while (next)
    {
        double delta = next->timeout - pulseCurrent;
        if (delta < PRECISION_TIME_THRESHOLD)
        {
            ProtoTimer* current = next;
            next = next->next;
            RemoveLongTimer(current);
            current->timeout = PrecisionTimeTotal(delta, GetPrecisionTime());
            InsertShortTimer(current);
        }
        else
        {
            break;
        }
    }
    if (NULL == long_head)
    {
        DeactivateTimer(pulse_timer);
        return false;
    }
    else
    {
        return true;
    }
}  // end ProtoTimerMgr::OnPulseTimeout()

void ProtoTimerMgr::ActivateTimer(ProtoTimer& theTimer)
{
    double timerInterval = theTimer.GetInterval();
    if (PRECISION_TIME_THRESHOLD > timerInterval)
    {
        double currentTime = GetPrecisionTime();
        //theTimer.timeout = PrecisionTimeTotal(currentTime, timerInterval); //ScenSim-Port://
        //Note: round up to micro-second precision //ScenSim-Port://
        theTimer.timeout = ceil(1.0e06 * PrecisionTimeTotal(currentTime, timerInterval))/1.0e06; //ScenSim-Port://
        InsertShortTimer(&theTimer);
    }
    else
    {
        if (!pulse_timer.IsActive())
        {
            pulse_mark = GetPrecisionTime();
            bool updateStatus = update_pending;
            update_pending = true;
            ActivateTimer(pulse_timer);
            update_pending = updateStatus;
        }
        theTimer.timeout = timerInterval + pulse_mark + 1.0 -
                           pulse_timer.GetTimeRemaining();
        InsertLongTimer(&theTimer);
    }
    theTimer.repeat_count = theTimer.repeat;
    if (!update_pending) Update();
}  // end ProtoTimerMgr::ActivateTimer()

void ProtoTimerMgr::ReactivateTimer(ProtoTimer* theTimer, double now)
{
    double timerInterval = theTimer->interval;
    if (PRECISION_TIME_THRESHOLD > timerInterval)
    {
        double theTimeout = PrecisionTimeTotal(theTimer->timeout, timerInterval);
        if (PrecisionTimeDelta(theTimeout, now) < -1.0)
        {
            theTimer->timeout = now;
            DMSG(0, "ProtoTimerMgr: Warning! real time failure\n");
        }
        else
        {
            theTimer->timeout = theTimeout;
        }
        InsertShortTimer(theTimer);
    }
    else
    {
        if (!pulse_timer.IsActive())
        {
            pulse_mark = GetPrecisionTime();
            bool updateStatus = update_pending;
            update_pending = true;
            ActivateTimer(pulse_timer);
            update_pending = updateStatus;
        }
        theTimer->timeout = timerInterval + GetPulseTime();
        InsertLongTimer(theTimer);
    }
    if (!update_pending) Update();
}  // end ProtoTimerMgr::ReactivateTimer(()

void ProtoTimerMgr::DeactivateTimer(ProtoTimer& theTimer)
{
    if (theTimer.mgr == this)
    {
        if (theTimer.is_precise)
        {
            RemoveShortTimer(&theTimer);
        }
        else
        {
            RemoveLongTimer(&theTimer);
            if (NULL == long_head)
            {
                bool updateStatus = update_pending;
                update_pending = true;
                DeactivateTimer(pulse_timer);
                update_pending = updateStatus;
            }
        }
        if (!update_pending) Update();
    }
}  // end ProtoTimerMgr::DeactivateTimer()

void ProtoTimerMgr::Update()
{
    double nextTimeout = NextTimeout();
    if (nextTimeout != scheduled_timeout)
    {
        if (nextTimeout < 0.0)
        {
            // Remove pending timeout
            UpdateSystemTimer(ProtoTimer::REMOVE, -1.0);
            scheduled_timeout = -1.0;
        }
        else
        {
            ProtoTimer::Command cmd = (scheduled_timeout < 0.0) ?
                                            ProtoTimer::INSTALL :
                                            ProtoTimer::MODIFY;
            double timeRemaining = GetTimeRemaining();
            if (UpdateSystemTimer(cmd, timeRemaining))
                scheduled_timeout = nextTimeout;
            else
                scheduled_timeout = -1.0;
        }
    }
}  // end ProtoTimerMgr::Update()

void ProtoTimerMgr::InsertShortTimer(ProtoTimer* theTimer)
{
    ProtoTimer* next  = short_head;
    theTimer->mgr = this;
    theTimer->is_precise = true;
    double theTimeout = theTimer->timeout;
    while(next)
    {
        if (PrecisionTimeDelta(theTimeout, next->timeout) <= 0.0)
        {
            theTimer->next = next;
            theTimer->prev = next->prev;
            if(next->prev != nullptr)
                theTimer->prev->next = theTimer;
            else
                short_head = theTimer;
            next->prev = theTimer;
            return;
        }
        else
        {
            next = next->next;
        }
    }

    theTimer->prev = short_tail;

    if (short_tail != nullptr)
        short_tail->next = theTimer;
    else
        short_head = theTimer;
    short_tail = theTimer;
    theTimer->next = NULL;
}  // end ProtoTimerMgr::InsertShortTimer()

void ProtoTimerMgr::RemoveShortTimer(ProtoTimer* theTimer)
{
    if (theTimer->prev)
        theTimer->prev->next = theTimer->next;
    else
        short_head = theTimer->next;
    if (theTimer->next)
        theTimer->next->prev = theTimer->prev;
    else
        short_tail = theTimer->prev;
    theTimer->mgr = NULL;
}  // end ProtoTimerMgr::RemoveShortTimer()

void ProtoTimerMgr::InsertLongTimer(ProtoTimer* theTimer)
{
    ProtoTimer* next  = long_head;
    theTimer->mgr = this;
    theTimer->is_precise = false;
    double theTimeout = theTimer->timeout;
    while(next)
    {
        if ((theTimeout - next->timeout) <= 0.0)
        {
            theTimer->next = next;
            theTimer->prev = next->prev;

            if(theTimer->prev != nullptr)
                theTimer->prev->next = theTimer;
            else
                long_head = theTimer;
            next->prev = theTimer;
            return;
        }
        else
        {
            next = next->next;
        }
    }
    theTimer->prev = long_tail;

    if (long_tail != nullptr)
        long_tail->next = theTimer;
    else
        long_head = theTimer;
    long_tail = theTimer;
    theTimer->next = NULL;
}  // end ProtoTimerMgr::InsertLongTimer()

void ProtoTimerMgr::RemoveLongTimer(ProtoTimer* theTimer)
{
    if (theTimer->prev)
        theTimer->prev->next = theTimer->next;
    else
        long_head = theTimer->next;
    if (theTimer->next)
        theTimer->next->prev = theTimer->prev;
    else
        long_tail = theTimer->prev;
    theTimer->mgr = NULL;
}  // end ProtoTimerMgr::RemoveLongTimer()



} //namespace// //ScenSim-Port://

