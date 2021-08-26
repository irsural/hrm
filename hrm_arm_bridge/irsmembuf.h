#ifndef IRSMEMBUFH
#define IRSMEMBUFH

#include <irsdefs.h>

#include <irscpp.h>
#include <mxdata.h>

#include <irsfinal.h>

namespace irs {

//! \ingroup in_out_group
//typedef<class container_type = vector<streambuf*> >
class membuf: public streambuf
{
public:
  typedef size_t size_type;
  typedef char char_type;
  // a_outbuf_size >= 4
  inline membuf(size_type a_outbuf_size = 1000);
  inline virtual ~membuf() {}
  inline virtual int overflow(int c = EOF);
  inline virtual int sync();
  inline const char_type* get_buf();
  inline size_type get_buf_size();
private:
  static const size_type m_reserv_char_size = 1;
  
  raw_data_t<char> m_outbuf;
  char_type* mp_buf_begin;
  char_type* mp_buf_end;
  char_type* mp_get_buf;
  size_t m_get_buf_size;
};
inline membuf::membuf(size_type a_outbuf_size):
  m_outbuf(a_outbuf_size),
  mp_buf_begin(m_outbuf.data() + a_outbuf_size/2),
  mp_buf_end(m_outbuf.data() + m_outbuf.size() - m_reserv_char_size),
  mp_get_buf(mp_buf_begin),
  m_get_buf_size(0)
{
  setp(mp_buf_begin, mp_buf_end);
}
inline int membuf::overflow(int c)
{
  if (c != EOF) {
    *pptr() = c;
    pbump(1);
  }
  int put_size = pptr() - pbase();
  int free_size = mp_get_buf - m_outbuf.data();
  if (put_size <= free_size) {
    m_get_buf_size = pptr() - mp_get_buf;
    memmoveex(mp_get_buf - put_size, mp_get_buf, m_get_buf_size);
    mp_get_buf -= put_size;
  } else {
    mp_get_buf += (put_size - free_size);
    m_get_buf_size = pptr() - mp_get_buf;
    memmoveex(mp_get_buf - put_size, mp_get_buf, m_get_buf_size);
    mp_get_buf = m_outbuf.data();
  }
  setp(mp_buf_begin, mp_buf_end);
  return 0;
}
inline int membuf::sync()
{
  return overflow();
}
inline const membuf::char_type* membuf::get_buf()
{
  *pptr() = 0;
  return mp_get_buf;
}
inline membuf::size_type membuf::get_buf_size()
{
  return m_get_buf_size;
}

} //namespace irs

#endif //IRSMEMBUFH
