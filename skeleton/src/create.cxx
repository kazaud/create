#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "create_config.h"

int main (int argc, char *argv[]) {
  fprintf(stdout, "Create version %d,%d\n", kCreateVersionMajor, kCreateVersionMinor);
  if (argc < 2) {
    fprintf(stdout,"Usage: %s number\n",argv[0]);
    return 1;
  }
  double inputValue = atof(argv[1]);
  double outputValue = sqrt(inputValue);
  fprintf(stdout,"The sqrxxxxxxxxxxxt of %g is %g\n", inputValue, outputValue);
  return 0;
}
