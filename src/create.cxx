#include <cstdio>
#include <cstdlib>
#include <cmath>

int main (int argc, char *argv[]) {
  if (argc < 2) {
    fprintf(stdout,"Usage: %s number\n",argv[0]);
    return 1;
  }
  double inputValue = atof(argv[1]);
  double outputValue = sqrt(inputValue);
  fprintf(stdout,"The sqrt of %g is %g\n", inputValue, outputValue);
  return 0;
}
