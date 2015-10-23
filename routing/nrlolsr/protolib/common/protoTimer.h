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
 *       by Brian Adamson and Joe Macker of the Naval Research 
 *       Laboratory (NRL)." 
 *         
 *  The name of NRL, the name(s) of NRL  employee(s), or any entity
 *  of the United States Government may not be used to endorse or
 *  promote  products derived from this software, nor does the 
 *  inclusion of the NRL written and developed software  directly or
 *  indirectly suggest NRL or United States  Government endorsement
 *  of this product.
 * 
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 ********************************************************************/


#ifndef _PROTO_TIMER
#define _PROTO_TIMER

#include "protoDefs.h"  // for ProtoSystemTime()
#include "protoDebug.h"


#include <math.h>



namespace NrlProtolibPort //ScenSim-Port://
{



class ProtoTimer
{
    friend class ProtoTimerMgr;
    
    public:
        // Construction/destruction
        ProtoTimer();
        ~ProtoTimer();
        
        // NOTE: For VC++ Debug builds, you _cannot_ use pre-compiled
        // headers with this template code.  Also, "/ZI" or "/Z7" compile options 
        // must NOT be specified.  (or else VC++ experiences an "internal compiler error")
        template <class listenerType>
        bool SetListener(listenerType* theListener, bool(listenerType::*timeoutHandler)(ProtoTimer&))
        {
            if (listener) delete listener;
            listener = theListener ? new LISTENER_TYPE<listenerType>(theListener, timeoutHandler) : NULL;
            return theListener ? (NULL != theListener) : true;
        }

        // Parameters
        // Timer interval (seconds)
        void SetInterval(double theInterval) 
            {interval = (theInterval < 0.0) ? 0.0 : theInterval;}
        double GetInterval() const {return interval;}
        // Timer repetition (0 =  one shot, -1 = infinite repetition)
        void SetRepeat(int numRepeat) {repeat = numRepeat;}
        int GetRepeat() const {return repeat;}
        void ResetRepeat() {repeat_count = repeat;}
        void DecrementRepeatCount() 
            {repeat_count = (repeat_count > 0) ? repeat_count-- : repeat_count;}
        int GetRepeatCount() {return repeat_count;}
        
        double GetTimeout() {return timeout;}
        
        // Activity status/control
        bool IsActive() const {return (NULL != mgr);}
        double GetTimeRemaining() const;
        bool Reschedule();
        void Scale(double factor);
        void Deactivate();
        
        // Ancillary
        void SetUserData(const void* userData) {user_data = userData;}
        const void* GetUserData() {return user_data;}
        
        // Installer commands
        enum Command {INSTALL, MODIFY, REMOVE};
            
    private:
        bool DoTimeout() {return listener ? listener->on_timeout(*this) : true;}
        class Listener
        {
            public:
                virtual ~Listener() {}
                virtual bool on_timeout(ProtoTimer& theTimer) = 0;
        };
        template <class listenerType>
        class LISTENER_TYPE : public Listener
        {
            public:
                LISTENER_TYPE(listenerType* theListener, bool(listenerType::*timeoutHandler)(ProtoTimer&))
                    : listener(theListener), timeout_handler(timeoutHandler) {}
                bool on_timeout(ProtoTimer& theTimer) {return (listener->*timeout_handler)(theTimer);}
                Listener* duplicate()
                    {return (static_cast<Listener*>(new LISTENER_TYPE<listenerType>(listener, timeout_handler)));}
            private:
                listenerType* listener;
                bool (listenerType::*timeout_handler)(ProtoTimer&);
        };
        Listener*                   listener;
        
        double                      interval;                
        int                         repeat;                  
        int                         repeat_count;            
        
        const void*                 user_data;
        double                      timeout;                 
        bool                        is_precise;              
        class ProtoTimerMgr*        mgr;                     
        ProtoTimer*                 prev;                    
        ProtoTimer*                 next;                    
};  // end class ProtoTimer


class ProtoTimerMgr
{
    friend class ProtoTimer;
    
    public:
        ProtoTimerMgr();
        virtual ~ProtoTimerMgr();
        
        // ProtoTimer activation/deactivation
        virtual void ActivateTimer(ProtoTimer& theTimer);
        virtual void DeactivateTimer(ProtoTimer& theTimer);
        
        // Status
        double GetTimeRemaining() const 
            {return (short_head ? short_head->GetTimeRemaining() : -1.0);}
        
        // Call this when the timer mgr's one-shot system timer fires
        void OnSystemTimeout();
        
        virtual void GetSystemTime(struct timeval& currentTime);
            
    protected:
        // System timer association/management definitions and routines
        virtual bool UpdateSystemTimer(ProtoTimer::Command command,
                                       double              delay) {return true;}
            
    private:
        // Methods used internally 
        void ReactivateTimer(ProtoTimer* theTimer, double now);
        void InsertLongTimer(ProtoTimer* theTimer);
        void RemoveLongTimer(ProtoTimer* theTimer);
        void InsertShortTimer(ProtoTimer* theTimer);
        void RemoveShortTimer(ProtoTimer* theTimer);
        void Update();
        double NextTimeout() const 
            {return (short_head ? short_head->timeout : -1.0);}
        double GetPulseTime() const
            {return (pulse_timer.IsActive() ? (pulse_mark + 1.0 - pulse_timer.GetTimeRemaining()) : -1.0);}
        bool OnPulseTimeout(ProtoTimer& theTimer);
        
        static const double PRECISION_TIME_THRESHOLD; 
        static const double PRECISION_TIME_MODULUS;  
        static const double PRECISION_TIME_HALF;
        static const long PRECISION_TIME_MASK;      
        double GetPrecisionTime()
        {
#ifdef _WIN32_WCE
            // We have found that overhead of "QueryPerformanceCounter()"
            // is a bit much at high rates, _and_ the "GetSystemTimeAsFileTime()"
            // approach of "::ProtoSystemTime()" is probably sufficient anyway
            // since we're using it in a "WaitForMultipleObjects()" call.
            // But, for WinCE, we need the precision of "QueryPerformanceCounter()"
            LARGE_INTEGER counterFreq = ProtoGetPerformanceCounterFrequency();
            if (0 != counterFreq.QuadPart)
            {
                LARGE_INTEGER count;
                QueryPerformanceCounter(&count);
                // (TBD) somehow correct possible precision time hiccup during rollover which
                // happens on at least some WinCE devices
                LARGE_INTEGER seconds;
                seconds.QuadPart = count.QuadPart / counterFreq.QuadPart;
                count.QuadPart -= (seconds.QuadPart * counterFreq.QuadPart);
                double fraction = (double)count.QuadPart / (double)counterFreq.QuadPart;
                seconds.LowPart &= PRECISION_TIME_MASK;
                return (((double)seconds.LowPart) + fraction);
            }
            else
#endif // _WIN32_WCE
            {
                struct timeval t;

//#if (defined(SIMULATE) && defined(NS2)) //ScenSim-Port:// 
#if (defined(SIMULATE) && (defined(NS2) || defined(SCENSIM_NRLOLSR))) //ScenSim-Port:// 
                GetSystemTime(t);
#else
                ::ProtoSystemTime(t);
#endif // if/else SIMULATE
                t.tv_sec &= PRECISION_TIME_MASK;
                double ptime = ((double)t.tv_sec + 1.0e-06*((double)t.tv_usec));
                return ptime;
            }
        }    
        // compute windowed t1 - t2
        static double PrecisionTimeDelta(double t1, double t2)
        {
            double delta = t1 - t2;
            return ((delta < -PRECISION_TIME_HALF) ?
                        (delta + PRECISION_TIME_MODULUS) :
                        (delta < PRECISION_TIME_HALF) ?
                            delta : (delta - PRECISION_TIME_MODULUS));
        } 
        // compute windowed t1 + t2
        static double PrecisionTimeTotal(double t1, double t2)
        {
            double total = t1 + t2;
            return ((total >= PRECISION_TIME_MODULUS) ? 
                        (total-PRECISION_TIME_MODULUS) : total);   
        }
        
        // Member variables
        bool            update_pending;                            
        double          scheduled_timeout;                                   
        ProtoTimer      pulse_timer;  // one second pulse timer    
        double          pulse_mark;                                
        ProtoTimer*     long_head;                                 
        ProtoTimer*     long_tail;                                 
        ProtoTimer*     short_head;                                
        ProtoTimer*     short_tail;  
};  // end class ProtoTimerMgr



} //namespace// //ScenSim-Port://


#endif // _PROTO_TIMER
