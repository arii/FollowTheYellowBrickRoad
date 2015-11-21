#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <inttypes.h>
#include <stddef.h>
#include "image_processing.h"

/*
 *conversion yuv to hsv. 0<h<360
 */
void YUVtoHSV(float* yuv,float* rgb,float* hsv)
{

float red,green,blue;
float rgbmin,rgbmax;

//https://innovativesolution.wordpress.com/2009/09/10/yuv-to-rgb-and-rgb-to-yuv-conversions/

red  = (yuv[0] - 0.0) + 1.596*(yuv[2] - 128.0);
green= (yuv[0] - 0.0) - 0.813*(yuv[1] - 128.0) - 0.3981*(yuv[2] - 128.0);
blue = (yuv[0] - 0.0) + 2.0172*(yuv[1] - 128.0);

if (red>255)   red=255;
if (green>255) green=255;
if (blue>255)  blue=255;
if (red<0)   red=0;
if (green<0) green=0;
if (blue<0)  blue=0;

rgbmax = red;
(rgbmax < green) && (rgbmax = green);
(rgbmax < blue) && (rgbmax = blue);

rgbmin = red;
(rgbmin > green) && (rgbmin = green);
(rgbmin > blue) && (rgbmin = blue);

//H
if (rgbmax == rgbmin)
	{
	hsv[0] = 0.0;
	}
else if (rgbmax==red)
	{
	hsv[0] = 60*(0+(green-blue)/(rgbmax-rgbmin));
	}
else if (rgbmax==green)
	{
	hsv[0] = 60*(2+(blue-red)/(rgbmax-rgbmin));
	}
else
	{
	hsv[0] = 60*(4+(red-green)/(rgbmax-rgbmin));
	}

if  (hsv[0]<0) {hsv[0]=hsv[0]+360.0;}

//S, V
(rgbmax==0) ? hsv[1]=0 : (hsv[1]=(rgbmax-rgbmin)/rgbmax);
hsv[2] = rgbmax/255;

rgb[0] = red;
rgb[1] = green;
rgb[2] = blue;

}



int main() {

printf("Simple test environment for image processing on Rolling spider \n");

//image variables
char   row[80];              /* for reading data */
char   in[80];               /* for reading data */
char   out[80];              /* for writing data */
float  **R;                  /* Red channel of RGB image */
float  **G;                  /* Green channel of RGB image */
float  **B;                  /* Blue channel of RGB image */
float  **Y;                  /* Y channel of YUV image */
float  **U;                  /* U channel of YUV image */
float  **V;                  /* V channel of YUV image */
float  **H;                  /* H channel of HSV image */
float  **S;                  /* S channel of HSV image */
float  **Va;                 /* Value channel of HSV image */
long   nx =0, ny=0;          /* image size in x, y direction */
int kimg;					 /*current image k*/

float yuv[3];
float rgb[3];
float hsv[3];
FILE   *inimage, *outimage;  /* input file, output file */
unsigned char byte;          /* for data conversion */

char filename[20];
char filetype[20];
char ffilename[20]="image.uyvy";

pixel2_t fileimage[80*120];

//helper variables
long   i, j;                 /* loop variables */
int lastImg;
int programMode;
float x_avg = 0;
int hits = 0;



//printf("ID of last image to process ? \n");
//scanf("%i", &lastImg);

/*
for (kimg=6;kimg<=lastImg;kimg=kimg+6)
	{


	sprintf(filename,"%s%i","../../DroneExchange/imgs/img",kimg);
	sprintf(filetype,"%s","bin");
	sprintf(ffilename,"%s.%s",filename,filetype);
	printf("ffilename: %s \n",ffilename);
	


	//open pgm file and read header
	
	if (filetype[0] == 'p')
	{		
		inimage = fopen(ffilename,"r");

		
		if (inimage = NULL)
			{
			printf("ERROR: no image %s  not found!\n",ffilename);
			exit(0);
			}


		fgets (row, 80, inimage);
		fgets (row, 80, inimage);
		while (row[0]=='#') fgets(row, 80, inimage);
		sscanf (row, "%ld %ld", &nx, &ny);
		printf("image size (nx,ny)=(%i,%i) \n",nx,ny);
		fgets (row, 80, inimage);

		if (nx==160)
			printf("WARNING: Image should be similar to 160x120 YUV422 interlaced, u.e. 80x120!\n");

		//allocate storage
		alloc_matrix (&R, nx+2, ny+2);
		alloc_matrix (&G, nx+2, ny+2);
		alloc_matrix (&B, nx+2, ny+2);
		alloc_matrix (&Y, nx+2, ny+2);
		alloc_matrix (&U, nx+2, ny+2);
		alloc_matrix (&V, nx+2, ny+2);
		alloc_matrix (&H, nx+2, ny+2);
		alloc_matrix (&S, nx+2, ny+2);
		alloc_matrix (&Va, nx+2, ny+2);

		//read image data
		for (j=0; j<ny; j++)
		 for (i=0; i<nx; i++)
		   {
			 R[i][j] = (float) getc (inimage);
			 G[i][j] = (float) getc (inimage);
			 B[i][j] = (float) getc (inimage);
		   }
		fclose(inimage);

	}

	//bin image, saved from drone
	else
	{*/
		 FILE* data;
		
		  if ((data = fopen(ffilename, "rb")) == NULL)
			{
				printf("ERROR opening file!\n");
				return 1;
			}
		
		nx=80;
		ny=120;


		fread(fileimage, sizeof(pixel2_t) *nx*ny, 1, data);
		
		fclose(data);
		
		//allocate storage

		alloc_matrix (&R, nx+2, ny+2);
		alloc_matrix (&G, nx+2, ny+2);
		alloc_matrix (&B, nx+2, ny+2);
		alloc_matrix (&Y, nx+2, ny+2);
		alloc_matrix (&U, nx+2, ny+2);
		alloc_matrix (&V, nx+2, ny+2);
		alloc_matrix (&H, nx+2, ny+2);
		alloc_matrix (&S, nx+2, ny+2);
		alloc_matrix (&Va, nx+2, ny+2);

		//read image data
		for (j=0; j<ny; j++)
		{
		 for (i=0; i<nx; i++)
		   {
			 Y[i][j] = (float)fileimage[nx*j+i].y1; //noneg yuv! transform to 0 centerd uav by (-16,-128,-128)
			 U[i][j] = (float)fileimage[nx*j+i].u;  //noneg yuv!
			 V[i][j] = (float)fileimage[nx*j+i].v;  //noneg yuv!

            /* if (Y[i][j] > (float)240){
                 x_avg += i;
                 hits++;
             }*/
			 //conversion

			 yuv[0] = Y[i][j];
			 yuv[1] = U[i][j];
			 yuv[2] = V[i][j];

			 YUVtoHSV(yuv,rgb,hsv); //this function takes non-zero yuv!

			 H[i][j] = hsv[0];
			 S[i][j] = hsv[1];
			 Va[i][j] = hsv[2];

			 R[i][j] = rgb[0];
			 G[i][j] = rgb[1];
			 B[i][j] = rgb[2];
             printf("%0.f,", R[i][j]);


             if (rgb[0]+rgb[1]+rgb[2] > 240*3){
                 x_avg += i;
                 hits++;
             }


		   }
         printf("\n");

		}

    if (hits > 0){
        x_avg = x_avg/hits;
        printf("x average is %f", x_avg);
    }
	}
    

