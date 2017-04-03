#include <irserror.h>
#pragma hdrstop

#include "imp_filt.h"

u309m::filt_imp_noise_t::filt_imp_noise_t(size_t a_max_size):
  m_rep()
{
  max_size(a_max_size);
}
u309m::filt_imp_noise_t::filt_imp_noise_t(
  const vector<value_type>& a_samples
):
  m_rep()
{
  assign(a_samples);
}
u309m::filt_imp_noise_t::rep_t::rep_t():
  sort_samples(),
  unsort_samples(),
  sort_samples_work(),
  max_size(0),
  result(0),
  sum(0),
  sum_pos_prev(0),
  //is_erased(false),
  is_first_calc(true),
  avg_prev(0),
  left_pos_prev_val(0),
  back_val(0),
  //it_left_pos_prev(),
  P_prev(0),
  signal_sort_samples(),
  signal_on(false)
{
}
size_t u309m::filt_imp_noise_t::size()
{
  return m_rep.sort_samples.size();
}
void u309m::filt_imp_noise_t::max_size(size_t a_max_size)
{
  m_rep.max_size = a_max_size;
}
size_t u309m::filt_imp_noise_t::max_size()
{
  return m_rep.max_size;
}
void u309m::filt_imp_noise_t::clear()
{
  m_rep.sort_samples.clear();
  m_rep.unsort_samples.clear();
  m_rep.sort_samples_work.clear();
  m_rep.sum = 0;
  m_rep.is_first_calc = true;
}
void u309m::filt_imp_noise_t::add_internal(value_type a_sample)
{
  sort_it_type it = m_rep.sort_samples.insert(a_sample);
  m_rep.unsort_samples.push_back(it);
  m_rep.sum += a_sample;
}
void u309m::filt_imp_noise_t::add(value_type a_sample)
{
  if (m_rep.unsort_samples.size() >= m_rep.max_size) {
    //m_rep.is_erased = true;
    sort_it_type it_erased = m_rep.unsort_samples.front();
    m_rep.sum -= *it_erased;
    m_rep.sort_samples.erase(it_erased);
    m_rep.unsort_samples.pop_front();
  }
  add_internal(a_sample);
  if (m_rep.unsort_samples.size() >= m_rep.max_size) {
    calc();
  }
}
void u309m::filt_imp_noise_t::assign(const vector<value_type>& a_samples)
{
  clear();
  typedef vector<value_type>::const_iterator it_t;
  it_t it_begin = a_samples.begin();
  it_t it_end = a_samples.end();
  for (it_t it = it_begin; it != it_end; ++it) {
    add_internal(*it);
  }
  calc();
}
void u309m::filt_imp_noise_t::calc()
{
  sort_cont_type& sort_samples_work = m_rep.sort_samples_work;
  sort_samples_work = m_rep.sort_samples;
  m_rep.is_first_calc = true;
  size_t samples_size = sort_samples_work.size();
  signal_sort_cont_type& signal_sort_samples = m_rep.signal_sort_samples;
  if (m_rep.signal_on) {
    for (size_t i = 0; i < samples_size; i++) {
      signal_sort_samples.insert(
        pair<value_type, size_t>(*m_rep.unsort_samples[i], i));
    }
  }

  value_type avg = 0;
  value_type sum = m_rep.sum;
  size_t loop_count = samples_size;

  for (size_t i = 0; i < loop_count; i++) {

    sort_it_type it_front = sort_samples_work.begin();
    sort_it_type it_back = sort_samples_work.end();
    --it_back;
    sum -= (*it_front + *it_back);
    avg = sum/(sort_samples_work.size() - 2);

    m_rep.back_val = *it_back;
    sort_samples_work.erase(it_front);
    sort_samples_work.erase(it_back);
    sort_it_type it1 = sort_samples_work.insert(avg);
    sort_it_type it2 = sort_samples_work.insert(avg);

    value_type avg_rectification = find_avg_rectification(avg);
    avg += avg_rectification;

    if (m_rep.signal_on) {
      signal_sort_it_type it_signal_front = signal_sort_samples.begin();
      signal_sort_it_type it_signal_back = signal_sort_samples.end();
      --it_signal_back;
      size_t idx_front = it_signal_front->second;
      size_t idx_back = it_signal_back->second;
      signal_sort_samples.erase(it_signal_front);
      signal_sort_samples.erase(it_signal_back);
      signal_sort_samples.insert(pair<value_type, size_t>(avg, idx_front));
      signal_sort_samples.insert(pair<value_type, size_t>(avg, idx_back));
    }

    sort_samples_work.erase(it1);
    sort_samples_work.erase(it2);
    sort_samples_work.insert(avg);
    sort_samples_work.insert(avg);

    sum += (avg + avg);
    m_rep.avg_prev = avg;

  }

  avg = sum/(sort_samples_work.size());

  m_rep.result = avg;
}
u309m::filt_imp_noise_t::value_type u309m::filt_imp_noise_t::
  find_avg_rectification(value_type a_avg)
{
  sort_cont_type& sort_samples_work = m_rep.sort_samples_work;
  range_type avg_range = sort_samples_work.equal_range(a_avg);
  sort_it_type& it_left_pos = avg_range.second;
  sort_it_type it_end = sort_samples_work.end();

  if (it_left_pos == it_end) {
    m_rep.is_first_calc = true;
    return 0;
  } else {
    sort_it_type it_left_pos_prev = it_end;
    value_type left_pos_prev_val = 0;
    bool is_full_calc = true;
    if (m_rep.is_first_calc) {
      m_rep.is_first_calc = false;
      is_full_calc = true;
    } else {
      // Учет входимости в старый диапазон текущего среднего и
      // предыдущего среднего после уточнения
      m_rep.P_prev--;
      value_type avg_prev = m_rep.avg_prev;
      left_pos_prev_val = m_rep.left_pos_prev_val;
      if (avg_prev >= left_pos_prev_val) {
        m_rep.sum_pos_prev += (avg_prev + avg_prev);
        m_rep.P_prev += 2;
      }
      if (a_avg >= left_pos_prev_val) {
        m_rep.sum_pos_prev += (a_avg + a_avg);
        m_rep.P_prev += 2;
      }
      it_left_pos_prev = sort_samples_work.lower_bound(left_pos_prev_val);
      if (it_left_pos_prev == it_end) {
        is_full_calc = true;
      } else {                      
        is_full_calc = false;
        m_rep.sum_pos_prev -= m_rep.back_val;
      }
    }

    size_t P = 0;
    value_type sum_pos = 0;  
    if (is_full_calc) {
      P = distance(avg_range.second, it_end);
      sum_pos = accumulate(avg_range.second, it_end, 0.);
      it_left_pos_prev = avg_range.second;
    } else {
      // Вычисление количества и суммы превысивших среднее
      // на основе предыдущих
      if (*it_left_pos > left_pos_prev_val) {
        P = m_rep.P_prev - distance(it_left_pos_prev, it_left_pos);
        sum_pos = m_rep.sum_pos_prev -
          accumulate(it_left_pos_prev, it_left_pos, 0.);  
      } else {
        P = m_rep.P_prev + distance(it_left_pos, it_left_pos_prev);
        sum_pos = m_rep.sum_pos_prev +
          accumulate(it_left_pos, it_left_pos_prev, 0.);
      }
    }
    m_rep.left_pos_prev_val = *it_left_pos;
    m_rep.P_prev = P;
    m_rep.sum_pos_prev = sum_pos;

    value_type Dtotal = sum_pos - P*a_avg;
    size_t samples_size = sort_samples_work.size();
    size_t avg_count = distance(avg_range.first, it_left_pos);
    ptrdiff_t deltaPN = 2*P - samples_size + avg_count;
    return (deltaPN*Dtotal)/(samples_size*samples_size);
  }
}
u309m::filt_imp_noise_t::value_type u309m::filt_imp_noise_t::get()
{
  return m_rep.result;
}
void u309m::filt_imp_noise_t::signal_on(bool a_on)
{
  m_rep.signal_on = a_on;
}
bool u309m::filt_imp_noise_t::signal_on()
{
  return m_rep.signal_on;
}
void u309m::filt_imp_noise_t::signal_get(signal_type* ap_signal)
{
  signal_sort_cont_type& signal_sort = m_rep.signal_sort_samples;
  size_t signal_size = signal_sort.size();
  ap_signal->resize(signal_size);
  signal_sort_it_type it_begin = signal_sort.begin();
  signal_sort_it_type it_end = signal_sort.end();
  for (signal_sort_it_type it = it_begin; it != it_end; ++it) {
    (*ap_signal)[it->second] = it->first;
  }
}
