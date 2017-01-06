#ifndef FIFO_HEADER
#define FIFO_HEADER 1

#if !FIFO_TEST
#include <avr/io.h>
#include <avr/interrupt.h>

#define START_CRITICAL_SECTION uint8_t _sreg = SREG; cli();
#define END_CRITICAL_SECTION SREG = _sreg;
#else
#define START_CRITICAL_SECTION 
#define END_CRITICAL_SECTION 
#ifndef uint8_t
typedef unsigned char uint8_t;
#endif
#endif

template<unsigned char sizeMask = 0x0f,typename DataT = uint8_t>
class FifoC {
public:
  FifoC()
    : m_head(0), // Where we're reading data from 
      m_tail(0)  // Place we're adding data.
  {}
  
  //! Compute the number of items in the queue.
  uint8_t Count() const {
    if(m_tail > m_head) 
      return m_tail - m_head;
    return m_tail + (sizeMask+1) - m_head;
  }
  
  //! Compute the amount of space left in the queue
  uint8_t Space() const
  { return sizeMask - Count(); }
  
  //! Test if the queue is empty.
  bool IsEmpty() const 
  { return m_head == m_tail; }
  
  //! Test if there is space in the queue
  bool IsSpace() const
  { return ((m_tail+1) & sizeMask) != m_head; }

  //! Add data to the tail of the queue.
  bool Put(const DataT &value) {
    START_CRITICAL_SECTION;
    // Compute next tail position.
    register uint8_t nextTail = (m_tail + 1) & sizeMask;
    // Check there's space.
    if(nextTail == m_head) {
      END_CRITICAL_SECTION;
      return false; // No space in queue.
    }
    // Put data into q.
    m_buf[m_tail] = value;
    m_tail = nextTail;
    END_CRITICAL_SECTION;
    return true;
  }

  //! Add data to the tail of the queue without disabling interrupts or checking size.
  void PutNoLock(const DataT &value) {
    m_buf[m_tail] = value;
    m_tail = (m_tail + 1) & sizeMask;
  }
  
  //! Get value from the head of the queue.
  DataT Get() {
    DataT ret =m_buf[m_head];
    m_head = (m_head + 1) & sizeMask;
    return ret;
  }
  
  //! Get value from the head of the queue.
  bool Get(DataT &value) {
    // Anything in queue ?
    if(m_head == m_tail)
      return false;
    
    // Yep, get next item.
    value = m_buf[m_head];
    m_head = (m_head + 1) & sizeMask;
    return true;
  }
  
  //! Access byte that get will return next.
  DataT Top() const
  { return m_buf[m_head]; }
  
protected:
  volatile uint8_t m_head;  // Next value to read
  volatile uint8_t m_tail;  // Next value to write.
  DataT m_buf[sizeMask+1];
};

#endif
