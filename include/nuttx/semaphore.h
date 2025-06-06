/****************************************************************************
 * include/nuttx/semaphore.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SEMAPHORE_H
#define __INCLUDE_NUTTX_SEMAPHORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <semaphore.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Initializers */

#ifdef CONFIG_PRIORITY_INHERITANCE
#  if CONFIG_SEM_PREALLOCHOLDERS > 0
/* semcount, flags, waitlist, hhead */

#    define NXSEM_INITIALIZER(c, f) \
       {{(c)}, (f), SEM_WAITLIST_INITIALIZER, NULL}
#  else
/* semcount, flags, waitlist, holder[2] */

#    define NXSEM_INITIALIZER(c, f) \
       {{(c)}, (f), SEM_WAITLIST_INITIALIZER, SEMHOLDER_INITIALIZER}
#  endif
#else /* CONFIG_PRIORITY_INHERITANCE */
/* semcount, flags, waitlist */

#  define NXSEM_INITIALIZER(c, f) \
     {{(c)}, (f), SEM_WAITLIST_INITIALIZER}
#endif /* CONFIG_PRIORITY_INHERITANCE */

/* Macros to retrieve sem count and to check if nxsem is mutex */

#define NXSEM_COUNT(s)        ((FAR atomic_t *)&(s)->val.semcount)
#define NXSEM_IS_MUTEX(s)     (((s)->flags & SEM_TYPE_MUTEX) != 0)

/* Mutex related helper macros */

#define NXSEM_MBLOCKING_BIT   (((uint32_t)1) << 31)
#define NXSEM_NO_MHOLDER      ((uint32_t)0x7ffffffe)
#define NXSEM_MRESET          ((uint32_t)0x7fffffff)

/* Macro to retrieve mutex's atomic holder's ptr */

#define NXSEM_MHOLDER(s)      ((FAR atomic_t *)&(s)->val.mholder)

/* Check if holder value (TID) is not NO_HOLDER or RESET */

#define NXSEM_MACQUIRED(h)    (((h) & NXSEM_NO_MHOLDER) != NXSEM_NO_MHOLDER)

/* Check if mutex is acquired and blocks some other task */

#define NXSEM_MBLOCKING(h)    (((h) & NXSEM_MBLOCKING_BIT) != 0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifdef CONFIG_FS_NAMED_SEMAPHORES
/* This is the named semaphore inode */

struct inode;
struct nsem_inode_s
{
  /* This must be the first element of the structure.  In sem_close() this
   * structure must be cast compatible with sem_t.
   */

  sem_t ns_sem;                     /* The contained semaphore */

  /* Inode payload unique to named semaphores. */

  FAR struct inode *ns_inode;       /* Containing inode */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_init
 *
 * Description:
 *   This function initializes the UNNAMED semaphore sem. Following a
 *   successful call to nxsem_init(), the semaphore may be used in subsequent
 *   calls to nxsem_wait(), nxsem_post(), and nxsem_trywait().  The semaphore
 *   remains usable until it is destroyed.
 *
 *   Only sem itself may be used for performing synchronization. The result
 *   of referring to copies of sem in calls to sem_wait(), sem_trywait(),
 *   sem_post(), and sem_destroy() is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be initialized
 *   pshared - Process sharing (not used)
 *   value - Semaphore initialization value
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_init(FAR sem_t *sem, int pshared, uint32_t value);

/****************************************************************************
 * Name: nxsem_destroy
 *
 * Description:
 *   This function is used to destroy the un-named semaphore indicated by
 *   'sem'.  Only a semaphore that was created using nxsem_init() may be
 *   destroyed using nxsem_destroy(); the effect of calling nxsem_destroy()
 *   with a named semaphore is undefined.  The effect of subsequent use of
 *   the semaphore sem is undefined until sem is re-initialized by another
 *   call to nxsem_init().
 *
 *   The effect of destroying a semaphore upon which other processes are
 *   currently blocked is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be destroyed.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_destroy(FAR sem_t *sem);

/****************************************************************************
 * Name: nxsem_wait / nxsem_wait_slow
 *
 * Description:
 *   This function attempts to lock the semaphore referenced by 'sem'.  If
 *   the semaphore value is (<=) zero, then the calling task will not return
 *   until it successfully acquires the lock.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     EINVAL - Invalid attempt to get the semaphore
 *     EINTR  - The wait was interrupted by the receipt of a signal.
 *
 ****************************************************************************/

int nxsem_wait(FAR sem_t *sem);
int nxsem_wait_slow(FAR sem_t *sem);

/****************************************************************************
 * Name: nxsem_trywait / nxsem_trywait_slow
 *
 * Description:
 *   This function locks the specified semaphore only if the semaphore is
 *   currently not locked.  Otherwise, it locks the semaphore.  In either
 *   case, the call returns without blocking.
 *
 * Input Parameters:
 *   sem - the semaphore descriptor
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     EINVAL - Invalid attempt to get the semaphore
 *     EAGAIN - The semaphore is not available.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsem_trywait(FAR sem_t *sem);
int nxsem_trywait_slow(FAR sem_t *sem);

/****************************************************************************
 * Name: nxsem_timedwait
 *
 * Description:
 *   This function will lock the semaphore referenced by sem as in the
 *   sem_wait() function. However, if the semaphore cannot be locked without
 *   waiting for another process or thread to unlock the semaphore by
 *   performing a sem_post() function, this wait will be terminated when the
 *   specified timeout expires.
 *
 *   The timeout will expire when the absolute time specified by abstime
 *   passes, as measured by the clock on which timeouts are based (that is,
 *   when the value of that clock equals or exceeds abstime), or if the
 *   absolute time specified by abstime has already been passed at the
 *   time of the call.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   That may be one of:
 *
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   EINTR     A signal interrupted this function.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int nxsem_timedwait(FAR sem_t *sem, FAR const struct timespec *abstime);

/****************************************************************************
 * Name: nxsem_clockwait
 *
 * Description:
 *   This function will lock the semaphore referenced by sem as in the
 *   sem_wait() function. However, if the semaphore cannot be locked without
 *   waiting for another process or thread to unlock the semaphore by
 *   performing a sem_post() function, this wait will be terminated when the
 *   specified timeout expires.
 *
 *   The timeout will expire when the absolute time specified by abstime
 *   passes, as measured by the clock on which timeouts are based (that is,
 *   when the value of that clock equals or exceeds abstime), or if the
 *   absolute time specified by abstime has already been passed at the
 *   time of the call.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   clockid - The timing source to use in the conversion
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   That may be one of:
 *
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   EINTR     A signal interrupted this function.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int nxsem_clockwait(FAR sem_t *sem, clockid_t clockid,
                    FAR const struct timespec *abstime);

/****************************************************************************
 * Name: nxsem_tickwait
 *
 * Description:
 *   This function is a lighter weight version of sem_timedwait().  It is
 *   non-standard and intended only for use within the RTOS.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to sem_trywait().
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure:
 *
 *     -ETIMEDOUT is returned on the timeout condition.
 *     -ECANCELED may be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int nxsem_tickwait(FAR sem_t *sem, uint32_t delay);

/****************************************************************************
 * Name: nxsem_post / nxsem_post_slow
 *
 * Description:
 *   When a kernel thread has finished with a semaphore, it will call
 *   nxsem_post().  This function unlocks the semaphore referenced by sem
 *   by performing the semaphore unlock operation on that semaphore.
 *
 *   If the semaphore value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the semaphore to become unlocked; the
 *   semaphore is simply incremented.
 *
 *   If the value of the semaphore resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the semaphore shall be
 *   allowed to return successfully from its call to sem_wait().
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int nxsem_post(FAR sem_t *sem);
int nxsem_post_slow(FAR sem_t *sem);

/****************************************************************************
 * Name:  nxsem_get_value
 *
 * Description:
 *   This function updates the location referenced by 'sval' argument to
 *   have the value of the semaphore referenced by 'sem' without effecting
 *   the state of the semaphore.  The updated value represents the actual
 *   semaphore value that occurred at some unspecified time during the call,
 *   but may not reflect the actual value of the semaphore when it is
 *   returned to the calling task.
 *
 *   If 'sem' is locked, the value return by nxsem_get_value() will either be
 *   zero or a negative number whose absolute value represents the number
 *   of tasks waiting for the semaphore.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *   sval - Buffer by which the value is returned
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_get_value(FAR sem_t *sem, FAR int *sval);

/****************************************************************************
 * Name: nxsem_open
 *
 * Description:
 *   This function establishes a connection between named semaphores and a
 *   task.  Following a call to sem_open() with the semaphore name, the task
 *   may reference the semaphore associated with name using the address
 *   returned by this call.  The semaphore may be used in subsequent calls
 *   to sem_wait(), sem_trywait(), and sem_post().  The semaphore remains
 *   usable until the semaphore is closed by a successful call to
 *   sem_close().
 *
 *   If a task makes multiple calls to sem_open() with the same name, then
 *   the same semaphore address is returned (provided there have been no
 *   calls to sem_unlink()).
 *
 * Input Parameters:
 *   sem    - Location to return the semaphore reference.
 *   name   - Semaphore name.
 *   oflags - Semaphore creation options.  This may either or both of the
 *     following bit settings.
 *     oflags = 0:  Connect to the semaphore only if it already exists.
 *     oflags = O_CREAT:  Connect to the semaphore if it exists, otherwise
 *        create the semaphore.
 *     oflags = O_CREAT|O_EXCL:  Create a new semaphore
 *        unless one of this name already exists.
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *     parameters are expected:
 *     1. mode_t mode, and
 *     2. unsigned int value.  This initial value of the semaphore. Valid
 *        initial values of the semaphore must be less than or equal to
 *        SEM_VALUE_MAX.
 *
 * Returned Value:
 *   0 (OK), or negated errno if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsem_open(FAR sem_t **sem, FAR const char *name, int oflags, ...);

/****************************************************************************
 * Name:  nxsem_close
 *
 * Description:
 *   This function is called to indicate that the calling task is finished
 *   with the specified named semaphore, 'sem'.  The sem_close() deallocates
 *   any system resources allocated by the system for this named semaphore.
 *
 *   If the semaphore has not been removed with a call to sem_unlink(), then
 *   sem_close() has no effect on the named semaphore.  However, when the
 *   named semaphore has been fully unlinked, the semaphore will vanish when
 *   the last task closes it.
 *
 * Input Parameters:
 *  sem - semaphore descriptor
 *
 * Returned Value:
 *  0 (OK), or negated errno if unsuccessful.
 *
 * Assumptions:
 *   - Care must be taken to avoid risking the deletion of a semaphore that
 *     another calling task has already locked.
 *   - sem_close must not be called for an un-named semaphore
 *
 ****************************************************************************/

int nxsem_close(FAR sem_t *sem);

/****************************************************************************
 * Name: nxsem_unlink
 *
 * Description:
 *   This function removes the semaphore named by the input parameter 'name.'
 *   If the semaphore named by 'name' is currently referenced by other task,
 *   the sem_unlink() will have no effect on the state of the semaphore.  If
 *   one or more processes have the semaphore open when sem_unlink() is
 *   called, destruction of the semaphore will be postponed until all
 *   references to the semaphore have been destroyed by calls of sem_close().
 *
 * Input Parameters:
 *   name - Semaphore name
 *
 * Returned Value:
 *  0 (OK), or negated errno if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsem_unlink(FAR const char *name);

/****************************************************************************
 * Name: nxsem_reset
 *
 * Description:
 *   Reset a semaphore count to a specific value.  This is similar to part
 *   of the operation of nxsem_init().  But nxsem_reset() may need to wake up
 *   tasks waiting on a count.  This kind of operation is sometimes required
 *   within the OS (only) for certain error handling conditions.
 *
 * Input Parameters:
 *   sem   - Semaphore descriptor to be reset
 *   count - The requested semaphore count
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_reset(FAR sem_t *sem, int16_t count);

/****************************************************************************
 * Name: nxsem_get_protocol
 *
 * Description:
 *    Return the value of the semaphore protocol attribute.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               queried.
 *    protocol - The user provided location in which to store the protocol
 *               value.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

#define nxsem_get_protocol(s,p) sem_getprotocol(s,p)

/****************************************************************************
 * Name: nxsem_set_protocol
 *
 * Description:
 *    Set semaphore protocol attribute.
 *
 *    One particularly important use of this function is when a semaphore
 *    is used for inter-task communication like:
 *
 *      TASK A                 TASK B
 *      sem_init(sem, 0, 0);
 *      sem_wait(sem);
 *                             sem_post(sem);
 *      Awakens as holder
 *
 *    In this case priority inheritance can interfere with the operation of
 *    the semaphore.  The problem is that when TASK A is restarted it is a
 *    holder of the semaphore.  However, it never calls sem_post(sem) so it
 *    becomes *permanently* a holder of the semaphore and may have its
 *    priority boosted when any other task tries to acquire the semaphore.
 *
 *    The fix is to call nxsem_set_protocol(SEM_PRIO_NONE) immediately after
 *    the sem_init() call so that there will be no priority inheritance
 *    operations on this semaphore.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               modified
 *    protocol - The new protocol to use
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_set_protocol(FAR sem_t *sem, int protocol);

/****************************************************************************
 * Name: nxsem_wait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_wait(), which is
 *   uninterruptible and convenient for use.
 *
 * Parameters:
 *   sem - Semaphore descriptor.
 *
 * Return Value:
 *   Zero(OK)  - On success
 *   EINVAL    - Invalid attempt to get the semaphore
 *   ECANCELED - May be returned if the thread is canceled while waiting.
 *
 * NOTE:  It is essential that callers of this function handle the
 * ECANCELED error.  Correct handling is that the function should return the
 * error and the error should propagate back up the calling tree to the
 * cancellation point interface function where the thread termination will
 * be handled gracefully
 *
 ****************************************************************************/

int nxsem_wait_uninterruptible(FAR sem_t *sem);

/****************************************************************************
 * Name: nxsem_timedwait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_timedwait(), which is
 *   uninterruptible and convenient for use.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 * NOTE:  It is essential that callers of this function handle the
 * ECANCELED error.  Correct handling is that the function should return the
 * error and the error should propagate back up the calling tree to the
 * cancellation point interface function where the thread termination will
 * be handled gracefully
 *
 ****************************************************************************/

int nxsem_timedwait_uninterruptible(FAR sem_t *sem,
                                    FAR const struct timespec *abstime);

/****************************************************************************
 * Name: nxsem_clockwait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_clockwait(), which is
 *   uninterruptible and convenient for use.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   clockid - The timing source to use in the conversion
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 * NOTE:  It is essential that callers of this function handle the
 * ECANCELED error.  Correct handling is that the function should return the
 * error and the error should propagate back up the calling tree to the
 * cancellation point interface function where the thread termination will
 * be handled gracefully
 *
 ****************************************************************************/

int nxsem_clockwait_uninterruptible(FAR sem_t *sem, clockid_t clockid,
                                    FAR const struct timespec *abstime);

/****************************************************************************
 * Name: nxsem_tickwait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_tickwait(), which is
 *   uninterruptible and convenient for use.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to sem_trywait().
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure:
 *
 *     -ETIMEDOUT is returned on the timeout condition.
 *     -ECANCELED may be returned if the thread is canceled while waiting.
 *
 * NOTE:  It is essential that callers of this function handle the
 * ECANCELED error.  Correct handling is that the function should return the
 * error and the error should propagate back up the calling tree to the
 * cancellation point interface function where the thread termination will
 * be handled gracefully
 *
 ****************************************************************************/

int nxsem_tickwait_uninterruptible(FAR sem_t *sem, uint32_t delay);

/****************************************************************************
 * Name: nxsem_getprioceiling
 *
 * Description:
 *   This function attempts to get the priority ceiling of a semaphore.
 *
 * Input Parameters:
 *   sem          - A pointer to the semaphore whose attributes are to be
 *                  modified
 *   prioceiling  - Location to return the semaphore's priority ceiling
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

int nxsem_getprioceiling(FAR const sem_t *sem, FAR int *prioceiling);

/****************************************************************************
 * Name: nxsem_setprioceiling
 *
 * Description:
 *   Set the priority ceiling of a semaphore.
 *
 * Input Parameters:
 *   mutex       - The mutex in which to set the mutex priority ceiling.
 *   prioceiling - The mutex priority ceiling value to set.
 *   old_ceiling - Location to return the mutex ceiling priority set before.
 *
 * Return Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure
 *
 ****************************************************************************/

int nxsem_setprioceiling(FAR sem_t *sem, int prioceiling,
                         FAR int *old_ceiling);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_SEMAPHORE_H */
