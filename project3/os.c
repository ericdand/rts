#include <string.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#define F_CPU 16000000
#include "util/delay.h"
#include "os.h"
/**
 * \file os.c
 * \brief A Skeleton Implementation of an RTOS
 *
 * \mainpage A Skeleton Implementation of a "Full-Served" RTOS Model
 * This is an example of how to implement context-switching based on a
 * full-served model. That is, the RTOS is implemented by an independent
 * "kernel" task, which has its own stack and calls the appropriate kernel
 * function on behalf of the user task.
 *
 * \author Dr. Mantis Cheng
 * \date 29 September 2006
 *
 * ChangeLog: Modified by Alexander M. Hoole, October 2006.
 *			  -Rectified errors and enabled context switching.
 *			  -LED Testing code added for development (remove later).
 *
 * \section Implementation Note
 * This example uses the ATMEL AT90USB1287 instruction set as an example
 * for implementing the context switching mechanism.
 * This code is ready to be loaded onto an AT90USBKey.  Once loaded the
 * RTOS scheduling code will alternate lighting of the GREEN LED light on
 * LED D2 and D5 whenever the correspoing PING and PONG tasks are running.
 * (See the file "cswitch.S" for details.)
 */

// Declare a_main - this is going to be the first task created when we
// run our RTOS - will come from either remote or base.c
extern void a_main();

typedef void (*voidfuncptr) (void);      /* pointer to void f(void) */


/*===========
  * RTOS Internal
  *===========
  */

/**
  * This internal kernel function is the context switching mechanism.
  * It is done in a "funny" way in that it consists two halves: the top half
  * is called "Exit_Kernel()", and the bottom half is called "Enter_Kernel()".
  * When kernel calls this function, it starts the top half (i.e., exit). Right in
  * the middle, "Cp" is activated; as a result, Cp is running and the kernel is
  * suspended in the middle of this function. When Cp makes a system call,
  * it enters the kernel via the Enter_Kernel() software interrupt into
  * the middle of this function, where the kernel was suspended.
  * After executing the bottom half, the context of Cp is saved and the context
  * of the kernel is restore. Hence, when this function returns, kernel is active
  * again, but Cp is not running any more.
  * (See file "switch.S" for details.)
  */
extern void CSwitch();
extern void Exit_Kernel();    /* this is the same as CSwitch() */

/* Prototype */
void Task_Terminate(void);

/**
  * This external function could be implemented in two ways:
  *  1) as an external function call, which is called by Kernel API call stubs;
  *  2) as an inline macro which maps the call into a "software interrupt";
  *       as for the AVR processor, we could use the external interrupt feature,
  *       i.e., INT0 pin.
  *  Note: Interrupts are assumed to be disabled upon calling Enter_Kernel().
  *     This is the case if it is implemented by software interrupt. However,
  *     as an external function call, it must be done explicitly. When Enter_Kernel()
  *     returns, then interrupts will be re-enabled by Enter_Kernel().
  */
extern void Enter_Kernel();

#define Disable_Interrupt()		asm volatile ("cli"::)
#define Enable_Interrupt()		asm volatile ("sei"::)
#define KERNEL_DEBUG_PIN PL4
#define OS_ABORT_DEBUG_PORT PORTC


/**
  *  This is the set of states that a task can be in at any given time.
  */
typedef enum process_states
{
   DEAD = 0,
   READY,
   RUNNING,
   BLOCKED,
   SUSPENDED
} PROCESS_STATES;

/**
  * This is the set of kernel requests, i.e., a request code for each system call.
  */
typedef enum kernel_request_type
{
   NONE = 0,
   CREATE,
   NEXT,
   TERMINATE,
   NEXT_TIME,
   CHAN_INIT,
   CHAN_SEND,
   CHAN_RECV,
   CHAN_WRITE
} KERNEL_REQUEST_TYPE;

typedef enum priorities
{
  SYSTEM = 0,
  TIME,
  RR,
  IDLE_TASK
} PRIORITIES;

/**
  * Each task is represented by a process descriptor, which contains all
  * relevant information about this task. For convenience, we also store
  * the task's stack, i.e., its workspace, in here.
  */
typedef struct ProcessDescriptor
{
   PID pid;
   PRIORITIES py;
   volatile unsigned char *sp;   /* stack pointer into the "workSpace" */
   unsigned char workSpace[WORKSPACE];
   PROCESS_STATES state;
   voidfuncptr  code;   /* function to be executed as a task */
   KERNEL_REQUEST_TYPE request;
   int arg;
   int kernel_response;
   CHAN comm_chan;
   int kernel_chan_arg;

   // Atrributes for time-based tasks
   TICK period;
   TICK wcet;
   TICK offset;
   TICK next_schedule;
   TICK executed_ticks;


   PRIORITIES py_arg;
   TICK period_arg;
   TICK wcet_arg;
   TICK offset_arg;

} PD;

// Queue Implementation
typedef struct ReadyQueue
{
  volatile PD* queue[MAXPROCESS];
  volatile int count;
  volatile int front;
  volatile int end;
} RQ;

/**
  * This table contains ALL process descriptors. It doesn't matter what
  * state a task is in.
  */
static PD Process[MAXPROCESS];

// The ready queues - time based tasks can only ever have one task queued
RQ ReadyQRR = {.count = 0, .front = 0, .end = 0};

RQ ReadyQTime = {.count = 0, .front = 0, .end = 0};

RQ ReadyQSystem = {.count = 0, .front = 0, .end = 0};

RQ ReadyQIdle = {.count = 0, .front = 0, .end = 0};

volatile TICK current_tick = 0;


/**
  * The process descriptor of the currently RUNNING task.
  */
volatile static PD* Cp;

/**
  * Since this is a "full-served" model, the kernel is executing using its own
  * stack. We can allocate a new workspace for this kernel stack, or we can
  * use the stack of the "main()" function, i.e., the initial C runtime stack.
  * (Note: This and the following stack pointers are used primarily by the
  *   context switching code, i.e., CSwitch(), which is written in assembly
  *   language.)
  */
volatile unsigned char *KernelSp;

/**
  * This is a "shadow" copy of the stack pointer of "Cp", the currently
  * running task. During context switching, we need to save and restore
  * it into the appropriate process descriptor.
  */
volatile unsigned char *CurrentSp;

/** index to next task to run */
volatile static unsigned int NextP;

/** 1 if kernel has been started; 0 otherwise. */
volatile static unsigned int KernelActive;

/** number of tasks created so far */
volatile static unsigned int Tasks;

typedef enum ChannelState {
	NOT_INIT = 0,
	IDLE,
	SENDER_WAIT,
	RECEIVER_WAIT
} CHANNEL_STATE;

/**
  * Channel object for data transmission between tasks
  */
typedef struct channel {
	CHANNEL_STATE state;
	PD *sender;
	RQ receivers;
	int val;
} CHANNEL;

/**
  * This table contains ALL channels.
  */
static CHANNEL channels[MAXCHAN];

volatile static unsigned int chanCount;

typedef enum ErrorCodes {
	NO_ERROR = 0,
	ERROR_EXCEEDS_MAXPROCESS,
	ERROR_EXCEEDS_MAXCHAN,
	ERROR_PERIODIC_BLOCK_OP,
	ERROR_TOO_MANY_SENDERS,
  ERROR_PERIODIC_TASK_COLLISION,
  ERROR_WCET_VIOLATION
} ERROR_CODES;

/*
rudimentary queueing implementation. This is used to enqueue
and dequeue from the ready queues. If a ready queue is ever overfilled
it is a violation and we panic and exit. This should be implemented as a
circular array. We need 2 integer indexes, one represents the front, one the end.
When we enqueue, we move the end forwards, wrapping around. When we dequeue,
we move the front forwards, wrapping around.
*/
void enqueue(volatile RQ* q, volatile PD* p)
{
  if (q->count == MAXPROCESS){
    // #TODO make this error code useful!
    OS_Abort(12);
  }
  q->queue[q->end] = p;
  q->count++;
  q->end = (q->end + 1) % MAXPROCESS;
}

volatile PD* dequeue(volatile RQ* q)
{
  if (q->count == 0){
    OS_Abort(13);
  }
  volatile PD* result = q->queue[q->front];
  q->count--;
  q->front = (q->front + 1) % MAXPROCESS;
  return result;
}

volatile int count(volatile RQ* q)
{
  return q->count;
}

// Put the task on the correct ready queue, and set its state to ready
void setReady(volatile PD* p)
{
  switch (p->py){
    case IDLE_TASK:
      enqueue(&ReadyQIdle, p);
	  break;
    case RR:
      enqueue(&ReadyQRR, p);
      break;
    case TIME:
      enqueue(&ReadyQTime, p);
      // If we ever set a time-task ready when there is a time-task already
      // ready, we have a timing violation, so we abort with error code
      if(count(&ReadyQTime) > 1){
        OS_Abort(ERROR_PERIODIC_TASK_COLLISION);
      }
      break;
    case SYSTEM:
      enqueue(&ReadyQSystem, p);
      break;
    default:
      OS_Abort(124);
      break;
  }
  p->state = READY;
}

/**
 * When creating a new task, it is important to initialize its stack just like
 * it has called "Enter_Kernel()"; so that when we switch to it later, we
 * can just restore its execution context on its stack.
 * (See file "cswitch.S" for details.)
 */
PID Kernel_Create_Task_At( volatile PD *p, voidfuncptr f, int arg, PID pid, PRIORITIES py, TICK period, TICK wcet, TICK offset )
{
   unsigned char *sp;

   //Changed -2 to -1 to fix off by one error.
   sp = (unsigned char *) &(p->workSpace[WORKSPACE-1]);



   /*----BEGIN of NEW CODE----*/
   //Initialize the workspace (i.e., stack) and PD here!

   //Clear the contents of the workspace
   memset(&(p->workSpace),0,WORKSPACE);

   //Notice that we are placing the address (16-bit) of the functions
   //onto the stack in reverse byte order (least significant first, followed
   //by most significant).  This is because the "return" assembly instructions
   //(rtn and rti) pop addresses off in BIG ENDIAN (most sig. first, least sig.
   //second), even though the AT90 is LITTLE ENDIAN machine.

   //Store terminate at the bottom of stack to protect against stack underrun.
   *(unsigned char *)sp-- = ((unsigned int)Task_Terminate) & 0xff;
   *(unsigned char *)sp-- = (((unsigned int)Task_Terminate) >> 8) & 0xff;
   *(unsigned char *)sp-- = 0x00;

   //Place return address of function at bottom of stack
   *(unsigned char *)sp-- = ((unsigned int)f) & 0xff;
   *(unsigned char *)sp-- = (((unsigned int)f) >> 8) & 0xff;
   *(unsigned char *)sp-- = 0x00;

   //Place stack pointer at top of stack
   sp = sp - 34;
   // set enable interrupt
   *(unsigned char *)(sp+1) |= (1 << 7);

   p->sp = sp;		/* stack pointer into the "workSpace" */
   p->code = f;		/* function to be executed as a task */
   p->request = NONE;
   p->arg = arg;
   p->pid = pid;
   p->py = py;
   p->request = NONE;

   // time-based stuff
   //#TODO consider the current tick in this
   p->period = period;
   p->wcet = wcet;
   p->offset = offset;
   p->next_schedule = offset;
   p->executed_ticks = 0;

   /*----END of NEW CODE----*/

   if (py == TIME) {
	p->state = SUSPENDED;
   } else {
	   //put on ready queue
	   setReady(p);
   }

   return p->pid;

}


/**
  *  Create a new task
  */
static PID Kernel_Create_Task( voidfuncptr f, int arg, PRIORITIES py, TICK period, TICK wcet, TICK offset)
{
   int x;

   if (Tasks == MAXPROCESS) return 0;  /* Too many task! */

   /* find a DEAD PD that we can use  */
   for (x = 0; x < MAXPROCESS; x++) {
       if (Process[x].state == DEAD) break;
   }
   ++Tasks;
   return Kernel_Create_Task_At( &(Process[x]), f, arg, x+1, py, period, wcet, offset);
   // #TODO if we just created a system or Timed task, we should call scheduler
   //Dispatch(); // ??????????

}

/**
  * This internal kernel function is a part of the "scheduler". It chooses the
  * next task to run, i.e., Cp.
  */
static void Dispatch()
{
     /* find the next READY task
       * Note: if there is no READY task, then this will loop forever!.
       */
    //Make this work with priority
  //  while(Process[NextP].state != READY) {
  //     NextP = (NextP + 1) % MAXPROCESS;
  //  }


    if (count(&ReadyQSystem) > 0)
    {
      Cp = dequeue(&ReadyQSystem);
      //  Cp = &(Process[NextP]);
      CurrentSp = Cp->sp;
      Cp->state = RUNNING;
      return;
    }

    if (count(&ReadyQTime) > 0)
    {
      Cp = dequeue(&ReadyQTime);
      //  Cp = &(Process[NextP]);
      CurrentSp = Cp->sp;
      Cp->state = RUNNING;
      return;
    }

    if (count(&ReadyQRR) > 0)
    {
      Cp = dequeue(&ReadyQRR);
      //  Cp = &(Process[NextP]);
      CurrentSp = Cp->sp;
      Cp->state = RUNNING;
      return;
    }

    if (count(&ReadyQIdle) > 0)
    {
      Cp = dequeue(&ReadyQIdle);
      CurrentSp = Cp->sp;
      Cp->state = RUNNING;
      return;
    }

    // WE SHOULD NEVER BE HERE HOPEFULLY
    OS_Abort(24);

    // //  Cp = &(Process[NextP]);
    // CurrentSp = Cp->sp;
    // Cp->state = RUNNING;

    //  NextP = (NextP + 1) % MAXPROCESS;
}

/**
  * Initializes the channel and its values
  */
CHAN Kernel_Chan_Init()
{
	if (chanCount >= MAXCHAN) {
		// No available channels - Print error and NO-OP/continue
		OS_Abort(ERROR_EXCEEDS_MAXCHAN);
		return NULL;
	} else {
		channels[chanCount].state = IDLE;
		channels[chanCount].receivers.count = 0;
		channels[chanCount].receivers.front = NULL;
		channels[chanCount].receivers.end = NULL;
		return ++chanCount;
	}
}

void Kernel_Chan_Send()
{
	if (Cp->py == TIME) OS_Abort(ERROR_PERIODIC_BLOCK_OP);

    CHANNEL *chan = &(channels[Cp->comm_chan-1]);

    // Check that the channel has been initialized
    if (chan->state == NOT_INIT) OS_Abort(2);

    if (chan->state == SENDER_WAIT) OS_Abort(ERROR_TOO_MANY_SENDERS);

	chan->val = Cp->kernel_chan_arg;
    if (chan->state == RECEIVER_WAIT) {
		// Send value & remove recipient from queue
		while (count(&(chan->receivers)) > 0){
			PD *receiver = dequeue(&(chan->receivers));
			receiver->kernel_response = chan->val;
			setReady(receiver);
			if (receiver->py < Cp->py) {
				setReady(Cp);
				Dispatch();
			}
		}
		chan->state = IDLE;
	} else {
		// Wait for a receiver...
		chan->state = SENDER_WAIT;
		chan->sender = Cp;
		Cp->state = BLOCKED;
    }
}

void Kernel_Chan_Receive()
{
	if (Cp->py == TIME) OS_Abort(ERROR_PERIODIC_BLOCK_OP);

	CHANNEL *chan = &(channels[Cp->comm_chan-1]);

	// Check that the channel has been initialized
	if (chan->state == NOT_INIT) OS_Abort(2);

	if (chan->state == SENDER_WAIT) {
		Cp->kernel_response = chan->val;
		setReady(chan->sender);
		if (chan->sender->py < Cp->py) {
			setReady(Cp);
			Dispatch();
		}
		chan->sender = NULL;
		chan->state = IDLE;
	} else {
		enqueue(&(chan->receivers), Cp);
		chan->state = RECEIVER_WAIT;
		Cp->state = BLOCKED;
	}
}

void Kernel_Chan_Write()
{
    CHANNEL *chan = &(channels[Cp->comm_chan-1]);

	// Check that the channel has been initialized
	if (chan->state == NOT_INIT) OS_Abort(2);

	if (chan->state == SENDER_WAIT) OS_Abort(ERROR_TOO_MANY_SENDERS);

	// Only write if receivers waiting
	if (chan->state == RECEIVER_WAIT) {
		chan->val = Cp->kernel_chan_arg;
		// Send value & remove recipient from queue
		while (count(&(chan->receivers)) > 0){
			PD *receiver = dequeue(&(chan->receivers));
			receiver->kernel_response = chan->val;
			setReady(receiver);
			if (receiver->py < Cp->py) {
				setReady(Cp);
				Dispatch();
			}
		}
		chan->state = IDLE;
	}
}

/**
  * This internal kernel function is the "main" driving loop of this full-served
  * model architecture. Basically, on OS_Start(), the kernel repeatedly
  * requests the next user task's next system call and then invokes the
  * corresponding kernel function on its behalf.
  *
  * This is the main loop of our kernel, called by OS_Start().
  */
static void Next_Kernel_Request()
{
   Dispatch();  /* select a new task to run */

   while(1) {
       Cp->request = NONE; /* clear its request */

       /* activate this newly selected task */
       CurrentSp = Cp->sp;
       Exit_Kernel();    /* or CSwitch() */

       /* if this task makes a system call, it will return to here! */

        /* save the Cp's stack pointer */
       Cp->sp = CurrentSp;

      //#TODO need to implement suspend so a time based task can give up CPU to resume
      // on the correct tick. What this will look like:
            // all tasks call task_next to give up CPU
            // if task was timed, then it suspends
            // else behaves as it does now
            // Every tick when we are in the kernel, we increment a TICK counter
            // Then we check all tasks with py = TIME for what their next tick
            // should be. If there is one that should be scheduled this tick, we
            // call setReady on it, and then continue to the scheduler
            // NOTE: This should ONLY EVER happen on a KERNEL TICK and not if a user
            // process is making a kernel request, so this should be specific to the
            // NONE case below I think
       switch(Cp->request){
       case CREATE:
          //  PORTA |= (1<<PA0);
           Kernel_Create_Task( Cp->code, Cp->arg, Cp->py_arg, Cp->period_arg, Cp->wcet_arg, Cp->offset_arg);
           // If we just created a system or timed task, call dispatch
           if (Cp->py_arg < RR && Cp->py_arg < Cp->py){
             //#TODO we also set ready in kernel_create_task, but I think this
             // is correct as in create we set ready the new task, but this should
             // set ready the task we are about to context-switch out of
             setReady(Cp);
             Dispatch();
           }
          //  PORTA &= ~(1<<PA0);
           break;
       case NEXT:
	     case NONE:
        //  PORTA |= (1<<PA1);
           /* NONE could be caused by a timer interrupt */
          setReady(Cp);
          Dispatch();
          // PORTA &= ~(1<<PA1);
          break;
       case NEXT_TIME:
          //  PORTA |= (1<<PA2);
           Cp->executed_ticks = 0;
           Cp->state = SUSPENDED;
           Dispatch();
          //  PORTA &= ~(1<<PA2);
		   break;
       case TERMINATE:
        //  PORTA |= (1<<PA4);
          /* deallocate all resources used by this task */
          Cp->state = DEAD;
		  Tasks--;
          Dispatch();
          // PORTA &= ~(1<<PA4);
          break;
	   case CHAN_INIT:
        // PORTA |= (1<<PA5);
		    Cp->kernel_response = Kernel_Chan_Init();
	      // PORTA &= ~(1<<PA5);
        break;
	   case CHAN_SEND:
        // PORTA |= (1<<PA6);
	      Kernel_Chan_Send();
		  if (Cp->state == BLOCKED) Dispatch();
        // PORTA &= ~(1<<PA6);
	      break;
	   case CHAN_RECV:
      //  PORTA |= (1<<PA7);
	      Kernel_Chan_Receive();
		  if (Cp->state == BLOCKED) Dispatch();
        // PORTA &= ~(1<<PA7);
	      break;
	   case CHAN_WRITE:
      //  PORTA |= (1<<PA3);
	      Kernel_Chan_Write();
        // PORTA &= ~(1<<PA3);
	      break;
       default:
          /* Houston! we have a problem here! */
          break;
       }
    }
}

/*================
  * RTOS  API  and Stubs
  *================
  */

/**
  * This function initializes the RTOS and must be called before any other
  * system calls.
  */
void OS_Init()
{
   int x;

   Tasks = 0;
   KernelActive = 0;
   NextP = 0;
	//Reminder: Clear the memory for the task on creation.
   for (x = 0; x < MAXPROCESS; x++) {
      memset(&(Process[x]),0,sizeof(PD));
      Process[x].state = DEAD;
   }

   // Channel memory allocation
   chanCount = 0;
   for (x = 0; x < MAXCHAN; x++) {
	   memset(&(channels[x]),0,sizeof(CHANNEL));
	   channels[x].state = NOT_INIT;
   }
}


/**
  * This function starts the RTOS after creating a few tasks.
  */
void OS_Start()
{
   if ( (! KernelActive) && (Tasks > 0)) {
       Disable_Interrupt();
      /* we may have to initialize the interrupt vector for Enter_Kernel() here. */

      /* here we go...  */
      KernelActive = 1;
      Next_Kernel_Request();
      /* NEVER RETURNS!!! */
   }
}

/**
  * TODO: communicate error code
  */
void OS_Abort(unsigned int error) {
	OS_ABORT_DEBUG_PORT = error;
	for(;;){}
}


/**
  * For this example, we only support cooperatively multitasking, i.e.,
  * each task gives up its share of the processor voluntarily by calling
  * Task_Next().
  */
PID Task_Create_RR( voidfuncptr f, int arg)
{
   if (KernelActive ) {
     Disable_Interrupt();
     Cp ->request = CREATE;
     Cp->code = f;
     Cp->arg = arg;
     Cp->py_arg = RR;
     Cp->period_arg = 0;
     Cp->wcet_arg = 0;
     Cp->offset_arg = 0;

     PORTL = (1<<KERNEL_DEBUG_PIN);
     Enter_Kernel();
   } else {
      /* call the RTOS function directly */
      Kernel_Create_Task( f, arg, RR, 0, 0, 0);
   }
   // #TODO this is a hack to return the PID. If an interrupt happens at
   // a bad time this could be wrong in theory..... IDK how to fix. API spec is weird
   return Tasks;
}

PID Task_Create_Period(voidfuncptr f, int arg, TICK period, TICK wcet, TICK offset)
{
  if (KernelActive ) {
    Disable_Interrupt();
    Cp ->request = CREATE;
    Cp->code = f;
    Cp->arg = arg;
    Cp->py_arg = TIME;

    Cp->period_arg = period;
    Cp->wcet_arg = wcet;
    Cp->offset_arg = offset;
    PORTL = (1<<KERNEL_DEBUG_PIN);
    Enter_Kernel();
  } else {
     /* call the RTOS function directly */
     Kernel_Create_Task( f, arg, TIME, period, wcet, offset);
  }
  return Cp->pid;
}

PID Task_Create_System(voidfuncptr f, int arg)
{
  if (KernelActive ) {
    Disable_Interrupt();
    Cp ->request = CREATE;
    Cp->code = f;
    Cp->arg = arg;
    Cp->py_arg = SYSTEM;
    Cp->period_arg = 0;
    Cp->wcet_arg = 0;
    Cp->offset_arg = 0;
    PORTL = (1<<KERNEL_DEBUG_PIN);
    Enter_Kernel();
  } else {
     /* call the RTOS function directly */
     Kernel_Create_Task( f, arg, SYSTEM, 0, 0, 0);
  }
  return Cp->pid;
}

PID Task_Create_Idle( voidfuncptr f, int arg)
{
   if (KernelActive ) {
     Disable_Interrupt();
     Cp ->request = CREATE;
     Cp->code = f;
     Cp->arg = arg;
     Cp->py = IDLE_TASK;
     Cp->period = 0;
     Cp->wcet = 0;
     Cp->offset = 0;

     PORTL = (1<<KERNEL_DEBUG_PIN);
     Enter_Kernel();
   } else {
      /* call the RTOS function directly */
      Kernel_Create_Task( f, arg, IDLE_TASK, 0, 0, 0);
   }
   return Cp->pid;
}

/**
  * The calling task gives up its share of the processor voluntarily.
  */
void Task_Next_2()
{
   if (KernelActive) {
     Disable_Interrupt();
     Cp ->request = NEXT;
     PORTL = (1<<KERNEL_DEBUG_PIN);
     Enter_Kernel();
  }
}

void Task_Next()
{
  if (KernelActive) {
    if(Cp->py != TIME){
      Task_Next_2();
    }else{
      // Here we handle the edge case of a Time based task giving up the
      // processor voluntarily. It should suspend itself
      Disable_Interrupt();
      Cp ->request = NEXT_TIME;
      PORTL = (1<<KERNEL_DEBUG_PIN);
      Enter_Kernel();
    }
  }
}

/**
  * The calling task gets its initial "argument" when it was created.
  */
int  Task_GetArg(void)
{
	return Cp->arg;
}


/**
  * The calling task terminates itself.
  */
void Task_Terminate()
{
   if (KernelActive) {
      Disable_Interrupt();
      Cp -> request = TERMINATE;
      PORTL = (1<<KERNEL_DEBUG_PIN);
      Enter_Kernel();
     /* never returns here! */
   }
}

/**
  * Requests channel through kernel
  * A value of zero/NULL means a channel could not be created
  */
CHAN Chan_Init()
{
	if (KernelActive) {
		Disable_Interrupt();
		Cp ->request = CHAN_INIT;
		PORTL = (1<<KERNEL_DEBUG_PIN);
		Enter_Kernel();
		return Cp->kernel_response;
	}
	return NULL;
}

/**
  * blocking send on CHAN
  */
void Send( CHAN ch, int v )
{
	if (KernelActive) {
		Disable_Interrupt();
		Cp->request = CHAN_SEND;
		Cp->comm_chan = ch;
		Cp->kernel_chan_arg = v;
		PORTL = (1<<KERNEL_DEBUG_PIN);
		Enter_Kernel();
	}
}

/**
  * blocking receive on CHAN
  */
int Recv( CHAN ch )
{
	if (KernelActive) {
		Disable_Interrupt();
		Cp->request = CHAN_RECV;
		Cp->comm_chan = ch;
		PORTL = (1<<KERNEL_DEBUG_PIN);
		Enter_Kernel();
		return Cp->kernel_response;
	}
	return (-1);
}

/**
  * non-blocking send on CHAN
  */
void Write( CHAN ch, int v )
{
	if (KernelActive) {
		Disable_Interrupt();
		Cp ->request = CHAN_WRITE;
		Cp->comm_chan = ch;
		Cp->kernel_chan_arg = v;
		PORTL = (1<<KERNEL_DEBUG_PIN);
		Enter_Kernel();
	}
}

/**
  * Returns number of milliseconds since RTOS boot
  * Each tick is 10 ms, and Timer3 resets every tick
  * From this, Current time = (10 * current_tick) + (Timer3 / 2000)
  * Note 2000 timer ticks = 1ms, with prescale 1/8
  */
unsigned int Now()
{
	unsigned int temp_time = TCNT3;
	unsigned int time = (10 * current_tick) + (temp_time / 2000);
	return time;
}


/*============
  * A Simple Test
  *============
  */
void Timer_Init()
{
  Disable_Interrupt();
  //Clear timer config.
  TCCR3A = 0;
  TCCR3B = 0;
  //Set to CTC (mode 4)
  TCCR3B |= (1<<WGM32);

  //Set prescaller to 1/8
  TCCR3B |= (1<<CS31);

  //Set TOP value 0.0001s*MSECPERTICK
  OCR3A = 2000*MSECPERTICK;

  //Enable interupt A for timer 3.
  TIMSK3 |= (1<<OCIE3A);

  //Set timer to 0 (optional here).
  TCNT3 = 0;

  // enable interrupt
  Enable_Interrupt();
}

void Kernel_Tick()
{
  current_tick++;
  int x;
  int ready_time_tasks = 0;
  for (x = 0; x < MAXPROCESS; x++) {
    if (Process[x].py == TIME && Process[x].state != SUSPENDED){
      Process[x].executed_ticks++;
      if (Process[x].executed_ticks >= Process[x].wcet){
        OS_Abort(ERROR_WCET_VIOLATION);
      }
    }
    if (Process[x].state == SUSPENDED && Process[x].py == TIME && Process[x].next_schedule == current_tick)
    {
      Process[x].next_schedule = Process[x].next_schedule + Process[x].period;
      setReady(&Process[x]);
    }
    if (Process[x].py == TIME && (Process[x].state == RUNNING || Process[x].state == READY))
    {
      ready_time_tasks++;
    }
  }
  if (ready_time_tasks > 1)
  {
    OS_Abort(ERROR_PERIODIC_TASK_COLLISION);
  }
  // if (Cp->py == TIME){
  //   Cp->executed_ticks++;
  //   if(Cp->executed_ticks >= Cp->wcet){
  //     OS_Abort(ERROR_WCET_VIOLATION);
  //   }
  // }

}

// This ISR fires every MSECPERTICKms and represents our RTOS tick
ISR(TIMER3_COMPA_vect)
{
  Kernel_Tick();
  if (Cp->py >= RR)
  {
    Task_Next_2();
  }
}

void Init_Debug_LEDs()
{
  DDRL |= (1<<PL2);
  DDRL |= (1<<PL3);
  DDRL |= (1<<PL4);
  DDRC = 0xFF;
  // DDRA |= (1<<PA0);
  // DDRA |= (1<<PA1);
  // DDRA |= (1<<PA2);
  // DDRA |= (1<<PA3);
  // DDRA |= (1<<PA4);
  // DDRA |= (1<<PA5);
  // DDRA |= (1<<PA6);
  // DDRA |= (1<<PA7);
  // DDRB |= (1<<PB0);
}

void Idle_Task()
{
  for(;;){}
}

/**
  * OS main function
  */
int main()
{
   OS_Init();
   Init_Debug_LEDs();
   // Here we create a task for a_main which should be defined externally to create
   // all tasks needed for the application, and then terminate.
   // #TODO this should be created as a system task once we implement this functionality
   Task_Create_Idle(Idle_Task, 0);
   Task_Create_System( a_main , PL2);
   Timer_Init();
   OS_Start();
}
