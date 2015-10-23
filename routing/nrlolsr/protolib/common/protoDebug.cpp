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


#include "protoDebug.h"

#include <stdio.h>   
#include <stdlib.h>  // for abort()
#include <stdarg.h>  // for variable args

#ifdef WIN32
#ifndef _WIN32_WCE
#include <io.h>
#include <fcntl.h>
#endif // ! _WIN32_CE
#include <Windows.h>
#include <tchar.h>
#endif // WIN32



namespace NrlProtolibPort //ScenSim-Port://
{ //ScenSim-Port://




#if defined(PROTO_DEBUG) || defined(PROTO_MSG)

static unsigned int debug_level = 0;
static FILE* debug_log = stderr;  // log to stderr by default

void SetDebugLevel(unsigned int level)
{
// LP 11-01-05 - replaced

// #ifdef WIN32
#if defined(WIN32) && !defined(SIMULATE)

// end LP
    if (level)
    {
        if (!debug_level && ((stderr == debug_log) || (stdout == debug_log))) 
            OpenDebugWindow();
    }
    else
    {
        if (debug_level && ((stderr == debug_log) || (debug_log == stdout))) 
            CloseDebugWindow();
    }
#endif // WIN32
    if(debug_log != NULL)
        //DMSG(0,"ProtoDebug>SetDebugLevel: Debug level changed from %d to %d\n", debug_level, level);//ScenSim-Port://
    debug_level = level;
}  // end SetDebugLevel()

unsigned int GetDebugLevel()
{
    return debug_level;
}

bool OpenDebugLog(const char *path)
{
    //DMSG(0,"ProtoDebug>OpenDebugLog: Debug log is being set to \"%s\"\n",path);//ScenSim-Port://
#ifdef OPNET  // JPH 4/26/06
    //printf ("OpenDebugLog: path = %s\n",path);
    if (!*path)
        return false;
#endif  // OPNET
    CloseDebugLog();
    FILE* ptr = fopen(path, "w+");
    if (ptr)
    {
// LP 11-01-05 - replaced
// #ifdef WIN32
    
#if defined(WIN32) && !defined(SIMULATE)
// end LP   
        if (debug_level && ((debug_log == stdout) || (debug_log == stderr))) 
            CloseDebugWindow();
#endif // WIN32
        debug_log = ptr;
        return true;
    }
    else
    {
// LP 11-01-05 - replaced
// #ifdef WIN32
#if defined(WIN32) && !defined(SIMULATE)
// end LP
        if (debug_level && (debug_log != stdout) && (debug_log != stderr)) 
            OpenDebugWindow();
#endif // WIN32
        debug_log = stderr;
        DMSG(0, "OpenDebugLog: Error opening debug log file: %s\n", path);
        return false;
    }
}  // end OpenLogFile()

void CloseDebugLog()
{
    if (debug_log && (debug_log != stderr) && (debug_log != stdout))
    {
        fclose(debug_log);
// LP 11-01-05 - replaced
// #ifdef WIN32
#if defined(WIN32) && !defined(SIMULATE)
// end LP
        if (debug_level) OpenDebugWindow();
#endif // WIN32
    }
    debug_log = stderr;
}

#if defined(WIN32) && !defined(SIMULATE)

static unsigned int console_reference_count = 0;

// Alternative WIN32 debug window (useful for WinCE)
// This is a simple window used instead of the
// normal console (which is not available in WinCE)

class ProtoDebugWindow
{
    public:
        ProtoDebugWindow();
        ~ProtoDebugWindow();

        bool IsOpen() {return (NULL != hwnd);}
        bool Create();
        void Destroy();
        void Print(const char* text, unsigned int len);

        void Popup() {if (hwnd) SetForegroundWindow(hwnd);}  // brings debug window to foreground
    
    private:
        static DWORD WINAPI RunInThread(LPVOID lpParameter);
        DWORD Run();
        static LRESULT CALLBACK MessageHandler(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);

        enum {BUFFER_MAX = 8190};
        HANDLE              thread_handle;
        DWORD               thread_id;
        DWORD               parent_thread_id;
        HWND                hwnd;
        char                buffer[BUFFER_MAX+2];
        unsigned int        count;
        CRITICAL_SECTION    buffer_section;
        int                 content_height;
        double              scroll_fraction;

};  // end class ProtoDebugWindow

ProtoDebugWindow::ProtoDebugWindow()
 : thread_handle(NULL), thread_id(NULL), parent_thread_id(NULL), 
   hwnd(NULL), count(0),
   content_height(0), scroll_fraction(0.0)
{
    
}

ProtoDebugWindow::~ProtoDebugWindow()
{            
    Destroy();
}

bool ProtoDebugWindow::Create()
{
    Destroy();
    parent_thread_id = GetCurrentThreadId();
    if (!(thread_handle = CreateThread(NULL, 0, RunInThread, this, 0, &thread_id)))
    {
        return true;
    }
    else
    {
        DMSG(0, "ProtoDebugWindow::Create() CreateThread() error: %s\n", GetErrorString());
        return false;
    }
}  // end ProtoDebugWindow::Create()

void ProtoDebugWindow::Destroy()
{
    parent_thread_id = NULL;
    if (NULL != thread_handle)
    {
        if (thread_id != GetCurrentThreadId())
        {
            if (hwnd) PostMessage(hwnd, WM_CLOSE, 0, 0);
            WaitForSingleObject(thread_handle, INFINITE);
            CloseHandle(thread_handle);
            thread_handle = NULL;
            thread_id = NULL;
        }
        else if (hwnd)
        {
            DestroyWindow(hwnd);
        }
    }
}  // end ProtoDebugWindow::Destroy()


DWORD WINAPI ProtoDebugWindow::RunInThread(LPVOID lpParameter)
{
    DWORD result = ((ProtoDebugWindow*)lpParameter)->Run();
    ExitThread(result);
    return result;
}

DWORD ProtoDebugWindow::Run()
{
    content_height = 0;
    scroll_fraction = 1.0;
    InitializeCriticalSection(&buffer_section);
    EnterCriticalSection(&buffer_section);
    memset(buffer, '\0', BUFFER_MAX+2);
    LeaveCriticalSection(&buffer_section);
    count = 0;

    HINSTANCE theInstance = GetModuleHandle(NULL);    
    // Register our msg_window class
    WNDCLASS cl;
    cl.style = CS_HREDRAW | CS_VREDRAW;
    cl.lpfnWndProc = MessageHandler;
    cl.cbClsExtra = 0;
    cl.cbWndExtra = 0;
    cl.hInstance = theInstance;
    cl.hIcon = NULL;
    cl.hCursor = NULL;
    cl.hbrBackground = NULL;
    cl.lpszMenuName = NULL;

    LPCTSTR myName = _T("ProtoDebugWindow"); // default name
    TCHAR moduleName[256];
    DWORD nameLen = GetModuleFileName(GetModuleHandle(NULL), moduleName, 256);
    if (0 != nameLen)
    {
        _tcsncat(moduleName, _T(" Debug"), 256 - nameLen);
        myName = moduleName;
    }

    
    cl.lpszClassName = myName;

    if (!RegisterClass(&cl))
    {
        LPVOID lpMsgBuf;
        FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | 
                      FORMAT_MESSAGE_FROM_SYSTEM | 
                      FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL, GetLastError(),
                      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
                      (LPTSTR) &lpMsgBuf, 0, NULL );
        // Display the string.
        MessageBox( NULL, (LPCTSTR)lpMsgBuf, (LPCTSTR)"Error", MB_OK | MB_ICONINFORMATION );
        // Free the buffer.
        LocalFree(lpMsgBuf);
        DeleteCriticalSection(&buffer_section);
        DMSG(0, "ProtoDebugWindow::Win32Init() Error registering message window class!\n");
        return GetLastError();
    }
    hwnd = CreateWindow(myName, 
                        myName,
                        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | 
                        WS_SIZEBOX | WS_VISIBLE | WS_VSCROLL,       
                        CW_USEDEFAULT, CW_USEDEFAULT,
                        CW_USEDEFAULT, CW_USEDEFAULT,
                        NULL, NULL, theInstance, this);
    if (NULL == hwnd)
    {
        Destroy();
        LPVOID lpMsgBuf;
        FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | 
                      FORMAT_MESSAGE_FROM_SYSTEM | 
                      FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL, GetLastError(), 0,
                     (LPTSTR)&lpMsgBuf, 0, NULL );
        // Display the string.
        MessageBox( NULL, (LPCTSTR)lpMsgBuf, (LPCTSTR)_T("Error"), MB_OK | MB_ICONINFORMATION);
        // Free the buffer.
        LocalFree(lpMsgBuf);
        UnregisterClass(cl.lpszClassName, theInstance);
        DeleteCriticalSection(&buffer_section);
        return GetLastError();
    }
    // Message loop
    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    UnregisterClass(cl.lpszClassName, theInstance);
    DeleteCriticalSection(&buffer_section);
    return msg.wParam;
}  // end ProtoDebugWindow::Run()



void ProtoDebugWindow::Print(const char* text, unsigned int len)
{
    ASSERT(NULL != hwnd);
    EnterCriticalSection(&buffer_section);
    if (len > BUFFER_MAX)
    {
        memcpy(buffer, text + len - BUFFER_MAX, BUFFER_MAX);
        count = BUFFER_MAX;
    }
    else
    {
        unsigned int space = BUFFER_MAX - count;
        unsigned int move = (len > space) ? len - space : 0;
        if (move) memmove(buffer, buffer+move, count - move);
        memcpy(buffer+count-move, text, len);
        count += len - move;
    }
#ifdef _UNICODE
    *((wchar_t*)(buffer+count)) = 0;
#else
    buffer[count] = '\0';
#endif // end if/else _UNICODE

    LeaveCriticalSection(&buffer_section);
    RECT rect;
    GetClientRect(hwnd, &rect);
    InvalidateRect(hwnd, &rect, FALSE);
    PostMessage(hwnd, WM_PAINT, 0, 0);
}  // end ProtoDebugWindow::Print()


LRESULT CALLBACK ProtoDebugWindow::MessageHandler(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message) 
    {
        case WM_CREATE:
        {
            CREATESTRUCT *info = (CREATESTRUCT*)lParam;
            ProtoDebugWindow* dbg = (ProtoDebugWindow*)info->lpCreateParams;
            SetWindowLong(hwnd, GWL_USERDATA, (DWORD)dbg);
            RECT rect;
            GetClientRect(hwnd, &rect);
            SetScrollRange(hwnd, SB_VERT, 0, 0, FALSE);
            SetScrollPos(hwnd, SB_VERT, 0, FALSE);
            return 0;
        }

        case WM_SIZE:
        {
            ProtoDebugWindow* dbg = (ProtoDebugWindow*)GetWindowLong(hwnd, GWL_USERDATA);
            int newHeight = HIWORD(lParam);
            if (dbg->content_height > newHeight)
            {
                SetScrollRange(hwnd, SB_VERT, 0, newHeight, TRUE);
            }
            else
            {
                SetScrollRange(hwnd, SB_VERT, 0, 0, TRUE);
                dbg->scroll_fraction = 1.0;
            }
            int pos = (int)((dbg->scroll_fraction * newHeight) + 0.5);
            SetScrollPos(hwnd, SB_VERT, pos, TRUE);
            return 0;
        }

        case WM_VSCROLL:
        {
            ProtoDebugWindow* dbg = (ProtoDebugWindow*)GetWindowLong(hwnd, GWL_USERDATA);
            RECT rect;
            GetClientRect(hwnd, &rect);
            switch (LOWORD(wParam))
            {
                case SB_BOTTOM:
                    SetScrollPos(hwnd, SB_VERT, rect.bottom, TRUE);
                    dbg->scroll_fraction = 1.0;
                    break;
                case SB_TOP:
                    SetScrollPos(hwnd, SB_VERT, 0, TRUE);
                    dbg->scroll_fraction = 0.0;
                    break;
                case SB_THUMBPOSITION:
                case SB_THUMBTRACK:
                {
                    int pos = HIWORD(wParam);
                    SetScrollPos(hwnd, SB_VERT, pos, TRUE);
                    dbg->scroll_fraction = (double)pos / rect.bottom;
                    break;
                }
                default:
                    break;
            }
            InvalidateRect(hwnd, &rect, FALSE);
            PostMessage(hwnd, WM_PAINT, 0, 0);
            return 0;
        }

        case WM_PAINT:
        {
            ProtoDebugWindow* dbg = (ProtoDebugWindow*)GetWindowLong(hwnd, GWL_USERDATA);
            char tempBuffer[BUFFER_MAX+2];
            EnterCriticalSection(&dbg->buffer_section);
            memcpy(tempBuffer, dbg->buffer, BUFFER_MAX+2);
            LeaveCriticalSection(&dbg->buffer_section);
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hwnd, &ps);
            RECT rect;
            GetClientRect(hwnd, &rect);
            RECT textRect = rect;
#ifndef _WIN32_WCE
            HFONT oldFont = (HFONT)SelectObject(hdc, GetStockObject(ANSI_FIXED_FONT));
#endif // !_WIN32_WCE
            HPEN oldPen = (HPEN)SelectObject(hdc, GetStockObject(BLACK_PEN));
            int textHeight = 
                DrawText(hdc, (LPCTSTR)tempBuffer, -1, &textRect, DT_CALCRECT | DT_LEFT | DT_NOPREFIX | DT_WORDBREAK);
            FillRect(hdc, &rect, (HBRUSH)GetStockObject(WHITE_BRUSH));
            int windowHeight = rect.bottom - rect.top;
            if (textHeight > windowHeight) 
            {
                int delta = textHeight - windowHeight;
                delta = (int)((delta * dbg->scroll_fraction) + 0.5);
                rect.top -= delta;
                bool updateScroll = dbg->content_height <= windowHeight;
                dbg->content_height = textHeight;
                SetScrollRange(hwnd, SB_VERT, 0, rect.bottom, TRUE);
            }
            DrawText(hdc, (LPCTSTR)tempBuffer, -1, &rect, DT_LEFT | DT_NOPREFIX | DT_WORDBREAK);
#ifndef _WIN32_WCE
            SelectObject(hdc, oldFont);
#endif // _WIN32_WCE
            SelectObject(hdc, oldPen);
            EndPaint(hwnd, &ps);
            return 0;
        }

        case WM_DESTROY:
        {
            ProtoDebugWindow* dbg = (ProtoDebugWindow*)GetWindowLong(hwnd, GWL_USERDATA);
            dbg->hwnd = NULL;
            console_reference_count = 0;
            PostQuitMessage(0);
            if (NULL != dbg->parent_thread_id)
                PostThreadMessage(dbg->parent_thread_id, WM_QUIT, 0, 0);
            return 0;
        }

        // (TBD) We could pick up WM_CLOSE to see if it's a user click or app command

        default:
            break;
    }
    return DefWindowProc(hwnd, message, wParam, lParam);
}  // end ProtoDebugWindow::MessageHandler()

#ifdef _WIN32_WCE
static ProtoDebugWindow debug_window;
#endif // _WIN32_WCE

#endif // WIN32


// Display string if statement's debug level is large enough
void DMSG(unsigned int level, const char *format, ...)
{
    if (level <= debug_level)
    {
        va_list args;
        va_start(args, format);
#ifdef _WIN32_WCE
        if (debug_window.IsOpen() && ((stderr == debug_log) || (stdout == debug_log)))
        {
            char charBuffer[2048];
            charBuffer[2048] = '\0';
            int count = _vsnprintf(charBuffer, 2047, format, args);
#ifdef _UNICODE
            wchar_t wideBuffer[2048];
            count = mbstowcs(wideBuffer, charBuffer, count);
            count *= sizeof(wchar_t);
            const char* theBuffer = (char*)wideBuffer;
#else
            const char* theBuffer = charBuffer;
#endif // if/else _UNICODE
            debug_window.Print(theBuffer, count);
        }
        else
#endif  // _WIN32_WCE
        {
            vfprintf(debug_log, format, args);
            fflush(debug_log);
        }
        va_end(args);   
    }
}  // end DMSG();

// LP 11-01-05 - replaced
// #ifdef WIN32
#if defined(WIN32) && !defined(SIMULATE)
// end LP

// Open console for displaying debug messages

void OpenDebugWindow()
{
    // Open a console window and redirect stderr and stdout to it
    if (0 == console_reference_count)
    {
#ifndef _WIN32_WCE
        AllocConsole();
        int hCrt = _open_osfhandle((long) GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
        FILE* hf = _fdopen(hCrt, "w" );
        *stdout = *hf;
        int i = setvbuf(stdout, NULL, _IONBF, 0 );
        
        hCrt = _open_osfhandle((long) GetStdHandle(STD_INPUT_HANDLE), _O_TEXT);
        hf = _fdopen(hCrt, "r" );
        *stdin = *hf;

        hCrt = _open_osfhandle((long) GetStdHandle(STD_ERROR_HANDLE), _O_TEXT);
        hf = _fdopen(hCrt, "w" );
        *stderr = *hf;
        i = setvbuf(stderr, NULL, _IONBF, 0);
#endif // if !_WIN32_CE
    }
#ifdef _WIN32_WCE
    if (!debug_window.IsOpen()) debug_window.Create();
#endif // _WIN32_WCE
    console_reference_count++;
}  // end OpenDebugWindow() 

#ifndef _WIN32_WCE
static HWND GetDebugWindowHandle(void)
{
   #define MY_BUFSIZE 1024 // Buffer size for console window titles.
   HWND hwndFound;         // This is what is returned to the caller.
   char pszNewWindowTitle[MY_BUFSIZE]; // Contains fabricated
                                       // WindowTitle.
   char pszOldWindowTitle[MY_BUFSIZE]; // Contains original
                                       // WindowTitle.
   // Fetch current window title
   GetConsoleTitle(pszOldWindowTitle, MY_BUFSIZE);
   // Format a "unique" NewWindowTitle.
   wsprintf(pszNewWindowTitle,"%d/%d",
               GetTickCount(),
               GetCurrentProcessId());
   // Change current window title.
   SetConsoleTitle(pszNewWindowTitle);
   // Ensure window title has been updated.
   Sleep(40);
   // Look for NewWindowTitle.
   hwndFound=FindWindow(NULL, pszNewWindowTitle);
   // Restore original window title.
   SetConsoleTitle(pszOldWindowTitle);
   return(hwndFound);
}  // end GetDebugWindowHandle()
#endif // !WIN32_WCE

void PopupDebugWindow() 
{
#ifdef _WIN32_WCE
    if (debug_window.IsOpen())
        debug_window.Popup();
    else
        OpenDebugWindow();
#else
    HWND debugWindow = GetDebugWindowHandle();
    if (debugWindow) 
        SetForegroundWindow(debugWindow);
    else
        OpenDebugWindow();
#endif  // if/else _WIN32_WCE
}

void CloseDebugWindow()
{
    if (console_reference_count > 0) console_reference_count--;
    if (0 == console_reference_count) 
    {
#ifdef _WIN32_WCE
        debug_window.Destroy();
#else
        FreeConsole();
#endif // if/else _WIN32_CE
    }
}
#endif // WIN32

#endif // PROTO_DEBUG || PROTO_MSG


#ifdef PROTO_DEBUG

void TRACE(const char *format, ...)
{
    va_list args;
    va_start(args, format);
#ifdef _WIN32_WCE
        if (debug_window.IsOpen() && ((stderr == debug_log) || (stdout == debug_log)))
        {
            char charBuffer[2048];
            charBuffer[2048] = '\0';
            int count = _vsnprintf(charBuffer, 2047, format, args);
#ifdef _UNICODE
            wchar_t wideBuffer[2048];
            count = mbstowcs(wideBuffer, charBuffer, count);
            count *= sizeof(wchar_t);
            const char* theBuffer = (char*)wideBuffer;
#else
            const char* theBuffer = charBuffer;
#endif // if/else _UNICODE
            debug_window.Print(theBuffer, count);
        }
        else
#endif  // _WIN32_WCE
    {
        vfprintf(debug_log, format, args);
        fflush(debug_log);
    }
    va_end(args);
}  // end TRACE();

#ifndef _WIN32_WCE
void ABORT(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vfprintf(debug_log, format, args);
    fflush(debug_log);
    va_end(args); 
    abort();
}
#endif // !_WIN32_WCE




#endif // if/else PROTO_DEBUG



} //namespace// //ScenSim-Port://



