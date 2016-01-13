#include "util.h"

vec mergesort(const vec &nums) {
  if (nums.size() == 1) {
    return nums;
  }
  vec a = mergesort(nums(span(0, nums.n_elem/2-1)));
  vec b = mergesort(nums(span(nums.n_elem/2, nums.n_elem-1)));
  uword i = 0, j = 0, k = 0;
  vec c(nums.n_elem, fill::zeros);
  while (k < c.n_elem) {
    if (i == a.n_elem) {
      c(k) = b(j); j++; k++;
    } else if (j == b.n_elem || a(i) < b(j)) {
      c(k) = a(i); i++; k++;
    } else {
      c(k) = b(j); j++; k++;
    }
  }
  return c;
}

double median_of_medians(const vec &nums) {
  vec tnums = nums;
  while (tnums.n_rows >= 15) {
    vec new_nums = vec((uword)ceil((double)tnums.n_elem / 5.0));
    for (uword i = 0; i < new_nums.n_elem; i++) {
      uword left = i * 5;
      uword right = (i + 1) * 5;
      if (right > tnums.n_elem) {
        right = tnums.n_elem;
      }
      vec S = mergesort(tnums(span(left, right - 1)));
      new_nums(i) = S(S.n_elem / 2);
    }
    printf("ayyyyy\n");
    tnums = new_nums;
  }
  vec S = mergesort(tnums);
  return S(S.n_elem / 2);
}
