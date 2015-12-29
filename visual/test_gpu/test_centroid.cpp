#include "gcube.h"
#include "gpu_util.h"
#include "highgui.h"
#include "imgproc.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    printf("usage: %s [image name]\n", argv[0]);
    return 1;
  }

  gcube I({ 1, 2, 3, 4, 5, 6, 7, 8, 9 });
  return 0;
}
