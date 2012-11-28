#include <stdio.h>
#include <math.h>

int main()
{
  FILE *fp;
  double a[12][5];
  double b[12][5];
  int i, j, k;
  double t_0 = 0;
  double t_f = 10.0;
  double Nsteps = 200;
  double T = 10.0;
  double t;
  double f;
  fp = fopen("/tmp/fourier_coefs.txt", "r");

  /* Need to read in fourier coefficients */
  for(i = 0; i < 12; i++) {
    for(j = 0; j < 5; j++) {
      fscanf(fp, "%lf", &a[i][j]);
    }
    for(j = 0; j < 5; j++) {
      fscanf(fp, "%lf", &b[i][j]);
    }
  }
  fclose(fp);
  
  /* Write the data file */
  fp = fopen ("data.txt", "w");
  for(i = 0; i < Nsteps; i++) {
    t = (i/Nsteps)*(t_f - t_0);
    fprintf(fp, "%lf ", t);
    for(j = 0; j < 12; j++) {
      f = 0.5 * a[j][0];
      for(k = 1; k < 5; k++) {
        f += a[j][k] * sin(2*M_PI*t*k/T);
        f += b[j][k] * cos(2*M_PI*t*k/T);
      }
      fprintf(fp, "%lf ", f);
    }
    fprintf(fp, "\n");
  }

  fclose(fp);
  return 0;
}
