#ifndef imp_filtH
#define imp_filtH

#include <irsdefs.h>

#include <irsdsp.h>
#include <irsalg.h>

#include <irsfinal.h>

namespace u309m {
  
class filt_imp_noise_t
{
public:
  typedef double value_type;
  typedef vector<value_type> signal_type;
  typedef signal_type::iterator signal_it_type;

  filt_imp_noise_t(size_t a_max_size = 0);
  filt_imp_noise_t(const vector<value_type>& a_samples);
  size_t size();
  // Класс работает при a_max_size >= 3
  void max_size(size_t a_max_size);
  size_t max_size();
  void clear();
  void add(value_type a_sample);
  void assign(const vector<value_type>& a_samples);
  value_type get();
  void signal_on(bool a_on);
  bool signal_on();
  void signal_get(signal_type* ap_signal);
private:
  typedef multiset<value_type> sort_cont_type;
  typedef sort_cont_type::iterator sort_it_type;
  typedef pair<sort_it_type, sort_it_type> range_type;
  typedef deque<sort_it_type> unsort_cont_type;
  //typedef unsort_cont_type::iterator unsort_it_type;
  typedef multimap<value_type, size_t> signal_sort_cont_type;
  typedef signal_sort_cont_type::iterator signal_sort_it_type;

  struct rep_t
  {
    rep_t();

    sort_cont_type sort_samples;
    unsort_cont_type unsort_samples;
    sort_cont_type sort_samples_work;
    size_t max_size;
    value_type result;
    value_type sum;
    value_type sum_pos_prev;
    //bool is_erased;
    bool is_first_calc;
    value_type avg_prev;
    value_type left_pos_prev_val;
    value_type back_val;
    //sort_it_type it_left_pos_prev;
    size_t P_prev;
    signal_sort_cont_type signal_sort_samples;
    bool signal_on;
  };

  rep_t m_rep;

  void add_internal(value_type a_sample);
  void calc();
  value_type find_avg_rectification(value_type a_avg);
};

} //  u309m

#endif  //  imp_filtH
