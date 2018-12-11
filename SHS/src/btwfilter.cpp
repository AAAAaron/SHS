/*
 * Copyright 2018 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */
/**********************************************
	使用描述，记得输入时.0不然就会按照int/int截断
	double wn[2] = { 2.5/(263.0/2), 0.0};
	double ab[4];
	double bb[4];
	int n = 4;
	mybutter(n, wn, 1, 0, ab, bb);
**********************************************/
#include "btwfilter.h"
namespace SHS {
  
typedef struct _C_double_complex

    {   /* double complex */

    double _Val[2];

    } _C_double_complex;
    
double pi = 3.1415926535897932384626433832795;

BtwFilter::BtwFilter()
{

}

BtwFilter::~BtwFilter()
{

}
/*************************************************************************
*tcexp @brief   :  复数的exp函数 For complex Z=X+i*Y, EXP(Z) = EXP(X)*(COS(Y)+i*SIN(Y))
* tconj@brief   : Complex conjugate.
%   CONJ(X) is the complex conjugate of X.
%   For a complex X, CONJ(X) = REAL(X) - i*IMAG(X).
*_Cmulcr复数乘以数 a+bi *c=ac+bci
*复数乘以复数 a+bi * c+di = (ac-bd)+i*(bc+ad)

* @inparam :  n		阶数
*			  
* @outparam:  temp		复数
*			  k		
* @author  :  tian
* @date    :  2018/10/18 18:40
* @version :  ver 1.0
*************************************************************************/
static _C_double_complex tcexp(_C_double_complex cdata)
{
	_C_double_complex temp;
	temp._Val[0]=exp(cdata._Val[0])*cos(cdata._Val[1]);
	temp._Val[1]=exp(cdata._Val[0])*sin(cdata._Val[1]);
	return temp;
}
static _C_double_complex tconj(_C_double_complex cdata)
{
	_C_double_complex temp;
	temp._Val[0]=cdata._Val[0];
	temp._Val[1]=-1*cdata._Val[1];
	return temp;
}
static double creal(_C_double_complex cdata)
{
	return cdata._Val[0];
}
static _C_double_complex _Cmulcr(_C_double_complex cdata,double rnum)
{
	_C_double_complex temp;
	temp._Val[0]=cdata._Val[0]*rnum;
	temp._Val[1]=cdata._Val[1]*rnum;
	return temp;

}
static _C_double_complex _Cmulcc(_C_double_complex cdata,_C_double_complex cdata2)
{
	_C_double_complex temp;
	temp._Val[0]=cdata._Val[0]*cdata2._Val[0]-cdata._Val[1]*cdata2._Val[1];
	temp._Val[1]=cdata._Val[1]*cdata2._Val[0]+cdata._Val[0]*cdata2._Val[1];
	return temp;

}
static double cabs(_C_double_complex cdata)
{
	return sqrt(pow(cdata._Val[0],2)+pow(cdata._Val[1],2));
}
/*************************************************************************
* @brief   :  Butterworth filter prototype, matlab函数
* @inparam :  n		阶数
*			  
* @outparam:  p		复矩阵
*			  k		
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
void mybuttap(int n, _C_double_complex* p, double *k)
{
	int i = 0, j = 0;
	int size = (n - 1) / 2;
	int isodd = n % 2;

	_C_double_complex temp;

	for (i = 1, j = 0; i < n; j++)
	{
		temp._Val[0] = 0;
		temp._Val[1] = pi * i / (2 * n) + pi / 2;
		p[j * 2] = tcexp(temp);
		
		i += 2;
	}

	for (int m = 1, i = 0; i < j; i++)
	{
		p[m] = tconj(p[m - 1]);
		m += 2;
	}

	if (isodd)
	{
		p[size * 2]._Val[0] = -1.0;
		p[size * 2]._Val[1] = 0.0;
	}

	_C_double_complex a = _Cmulcc(_Cmulcr(p[0], -1), _Cmulcr(p[1], -1));
	for (int m = 2; m < size * 2 + isodd; m++)
	{
		a = _Cmulcc(a, _Cmulcr(p[m], -1));
	}

	*k = creal(a);
}

/*************************************************************************
* @brief   :  Characteristic polynomial or polynomial with specified roots, matlab函数
* @inparam :  p		复矩阵
*			  np	复矩阵的大小
* @outparam:  d		返回复矩阵
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void mypoly(_C_double_complex* p, int np, _C_double_complex *d)
{
	_C_double_complex *c = (_C_double_complex *)malloc((np + 1) * sizeof(_C_double_complex));
	c[0]._Val[0] = 1.0;
	c[0]._Val[1] = 0.0;
	d[0]._Val[0] = 1.0;
	d[0]._Val[1] = 0.0;
	for (int i = 1; i<np + 1; i++)
	{
		c[i]._Val[0] = 0.0;
		c[i]._Val[1] = 0.0;
		d[i]._Val[0] = 0.0;
		d[i]._Val[1] = 0.0;
	}

	_C_double_complex temp;
	for (int i = 0; i < np; i++)
	{
		for (int j = 1; j <= i + 1; j++)
		{
			temp = _Cmulcc(p[i], d[j - 1]);
			c[j]._Val[0] = d[j]._Val[0] - temp._Val[0];
			c[j]._Val[1] = d[j]._Val[1] - temp._Val[1];
		}
		for (int j = 1; j <= i + 1; j++)
		{
			d[j]._Val[0] = c[j]._Val[0];
			d[j]._Val[1] = c[j]._Val[1];
		}
	}
	free(c);
}

/*************************************************************************
* @brief   :  实数矩阵相乘
* @inparam :  a		矩阵A
*			  b		矩阵B
*			  m		矩阵A与乘积矩阵C的行数
*			  n		矩阵A的行数,矩阵B的列数
*			  k		矩阵B与乘积矩阵C的列数
* @outparam:  c		乘积矩阵 C=AB 
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void mytrmul(double a[], double b[], int m, int n, int k, double c[])
{
	int i, j, l, u;
	for (i = 0; i <= m - 1; i++)
		for (j = 0; j <= k - 1; j++)
		{
			u = i*k + j; c[u] = 0.0;
			for (l = 0; l <= n - 1; l++)
				c[u] = c[u] + a[i*n + l] * b[l*k + j];
		}
}

/*************************************************************************
* @brief   :  矩阵求逆
* @inparam :  a		矩阵A
*			  n		矩阵A的阶数
* @outparam:  a		逆矩阵
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void myrinv(double a[], int n)
{
	int *is, *js, i, j, k, l, u, v;
	double d, p;
	is = (int *)malloc(n * sizeof(int));
	js = (int *)malloc(n * sizeof(int));

	for (k = 0; k <= n - 1; k++)
	{
		d = 0.0;
		for (i = k; i <= n - 1; i++)
			for (j = k; j <= n - 1; j++)
			{
				l = i*n + j; p = fabs(a[l]);
				if (p>d) { d = p; is[k] = i; js[k] = j; }
			}
		if (d + 1.0 == 1.0)
		{
			free(is); free(js);
		}
		if (is[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = is[k] * n + j;
				p = a[u]; a[u] = a[v]; a[v] = p;
			}
		if (js[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k; v = i*n + js[k];
				p = a[u]; a[u] = a[v]; a[v] = p;
			}
		l = k*n + k;
		a[l] = 1.0 / a[l];
		for (j = 0; j <= n - 1; j++)
			if (j != k)
			{
				u = k*n + j; a[u] = a[u] * a[l];
			}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
				for (j = 0; j <= n - 1; j++)
					if (j != k)
					{
						u = i*n + j;
						a[u] = a[u] - a[i*n + k] * a[k*n + j];
					}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
			{
				u = i*n + k; a[u] = -a[u] * a[l];
			}
	}
	for (k = n - 1; k >= 0; k--)
	{
		if (js[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = js[k] * n + j;
				p = a[u]; a[u] = a[v]; a[v] = p;
			}
		if (is[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k; v = i*n + is[k];
				p = a[u]; a[u] = a[v]; a[v] = p;
			}
	}
	free(is); free(js);
}

/*************************************************************************
* @brief   :  复矩阵求逆
* @inparam :  ar	矩阵A的实部
*			  ai	矩阵A的虚部
*			  n		矩阵A的阶数
* @outparam:  ar	逆矩阵的实部
*			  ai	逆矩阵的虚部
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
//复矩阵求逆
static int mycinv(double *ar, double *ai, int n)
{ 
	int *is, *js, i, j, k, l, u, v, w;
	double p, q, s, t, d, b;
	is = (int*)malloc(n * sizeof(int));
	js = (int*)malloc(n * sizeof(int));
	for (k = 0; k <= n - 1; k++)
	{
		d = 0.0;
		for (i = k; i <= n - 1; i++)
			for (j = k; j <= n - 1; j++)
			{
				u = i*n + j;
				p = ar[u] * ar[u] + ai[u] * ai[u];
				if (p>d) { d = p; is[k] = i; js[k] = j; }
			}
		if (d + 1.0 == 1.0)
		{
			free(is); free(js);
			return(0);
		}
		if (is[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = is[k] * n + j;
				t = ar[u]; ar[u] = ar[v]; ar[v] = t;
				t = ai[u]; ai[u] = ai[v]; ai[v] = t;
			}
		if (js[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k; v = i*n + js[k];
				t = ar[u]; ar[u] = ar[v]; ar[v] = t;
				t = ai[u]; ai[u] = ai[v]; ai[v] = t;
			}
		l = k*n + k;
		ar[l] = ar[l] / d; ai[l] = -ai[l] / d;
		for (j = 0; j <= n - 1; j++)
			if (j != k)
			{
				u = k*n + j;
				p = ar[u] * ar[l]; q = ai[u] * ai[l];
				s = (ar[u] + ai[u])*(ar[l] + ai[l]);
				ar[u] = p - q; ai[u] = s - p - q;
			}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
			{
				v = i*n + k;
				for (j = 0; j <= n - 1; j++)
					if (j != k)
					{
						u = k*n + j;  w = i*n + j;
						p = ar[u] * ar[v]; q = ai[u] * ai[v];
						s = (ar[u] + ai[u])*(ar[v] + ai[v]);
						t = p - q; b = s - p - q;
						ar[w] = ar[w] - t;
						ai[w] = ai[w] - b;
					}
			}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
			{
				u = i*n + k;
				p = ar[u] * ar[l]; q = ai[u] * ai[l];
				s = (ar[u] + ai[u])*(ar[l] + ai[l]);
				ar[u] = q - p; ai[u] = p + q - s;
			}
	}
	for (k = n - 1; k >= 0; k--)
	{
		if (js[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = js[k] * n + j;
				t = ar[u]; ar[u] = ar[v]; ar[v] = t;
				t = ai[u]; ai[u] = ai[v]; ai[v] = t;
			}
		if (is[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k; v = i*n + is[k];
				t = ar[u]; ar[u] = ar[v]; ar[v] = t;
				t = ai[u]; ai[u] = ai[v]; ai[v] = t;
			}
	}
	free(is); free(js);
	return(1);
}


/*************************************************************************
* @brief   :  对复数排序
* @inparam :  p			复矩阵
*			  n			复矩阵大小
* @outparam:  p
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void sort_complex(_C_double_complex *p, int n)
{
	_C_double_complex *pa, *pb, *k, temp;
	for (pa = p; pa<p + n - 1; pa++)
	{
		k = pa;
		for (pb = pa + 1; pb < p + n; pb++)
			if (creal(*k) > creal(*pb))
				k = pb;
		temp = *pa;
		*pa = *k;
		*k = temp;
	}
}

/*************************************************************************
* @brief   :  Convert zero-pole-gain filter parameters to state-space form,matlab函数
* @inparam :  np			复矩阵p的阶数
*			  p,k0
*			  a,b,c,d
* @outparam:  a,b,c,d
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void  myzp2ss(_C_double_complex* p, int np, double k0, double *a, double *b, double *c, double *d)
{
	int np1 = np;
	for (int i = 0; i<np*np; i++)
	{
		a[i] = 0.0;
	}
	for (int i = 0; i<np; i++)
	{
		b[i] = 0.0;
		c[i] = 0.0;
	}
	*d = 1;

	//If odd number of poles only, convert the pole at the
	//end into state-space.
	//H(s) = 1/(s-p1) = 1/(s + den(2)) 
	if (np % 2)
	{
		a[0] = -1;
		b[0] = 1;
		c[0] = 1;
		*d = 0;
		np1 = np - 1;
	}
	sort_complex(p, np1);

	//Take care of any left over unmatched pole pairs.
	//H(s) = 1/(s^2+den(2)s+den(3))
	_C_double_complex p_temp[2];
	_C_double_complex c_temp[3];
	double den[3], wn;
	double t[2 * 2] = { 0.0, 0.0, 0.0, 0.0 };
	double t_rinv[2 * 2] = { 0.0, 0.0, 0.0, 0.0 };
	double tr_den[2 * 2] = { 0.0, 0.0, 0.0, 0.0 };

	double a1_temp[2 * 2] = { 0.0, 0.0, 0.0, 0.0 };
	double a1[2 * 2] = { 0.0, 0.0, 0.0, 0.0 };
	double b1[2 * 2] = { 0.0, 0.0, 0.0, 0.0 };
	double c1[2 * 2] = { 0.0, 0.0, 0.0, 0.0 };
	double d1 = 0;
	int c_a = np % 2;	//a的列数
	int ma1 = np % 2;	//a的行数
	int na2 = 2;	//a1的列数	

	for (int i = 0; i < np1; i = i + 2)
	{
		p_temp[0] = p[i];
		p_temp[1] = p[i + 1];
		mypoly(p_temp, 2, c_temp);
		for (int j = 0; j<3; j++)
		{
			den[j] = creal(c_temp[j]);
		}
		wn = sqrt(cabs(p_temp[0]) * cabs(p_temp[1]));
//		wn = 1;
		if (wn == 0)
			wn = 1;
		//a1 = t\[-den(2) -den(3); 1 0]*t;
		t[0] = 1.0;				//t[0][0]
		t[1*2+1] = 1.0 / wn;	//t[1][1]
		t_rinv[0] = 1.0;
		t_rinv[1 * 2 + 1] = 1.0 / wn;
		myrinv(t_rinv, 2);
		tr_den[0] = -den[1];
		tr_den[0*2+1] = -den[2];
		tr_den[1*2+0] = 1.0;
		tr_den[1*2+1] = 0.0;
		mytrmul(t_rinv, tr_den, 2, 2, 2, a1_temp);
		mytrmul(a1_temp, t, 2, 2, 2, a1);
		//b1 = t\[1; 0];
		double tr_temp1[2 * 1] = { 1.0, 0.0 };
		mytrmul(t_rinv, tr_temp1, 2, 2, 1, b1);
		double tr_temp2[2] = { 0.0, 1.0 };
		mytrmul(tr_temp2, t_rinv, 1, 2, 2, c1);
		d1 = 0;

		//[a,b,c,d] = series(a,b,c,d,a1,b1,c1,d1);
		//Next lines perform series connection 
		if (ma1 != 0)
		{
			//a = [a zeros(ma1,na2); b1*c a1];
			for (int k = 0; k<ma1; k++)
			{
				for (int j = c_a; j<c_a + 2; j++)
				{
					a[k*np + j] = 0;
				}
			}			
			a[ma1*np+(c_a - 1)] = 1;
			for (int k = ma1, kk = 0; kk < 2; k++,kk++)
			{
				for (int j = c_a,jj=0; jj < 2; j++,jj++)
				{
					a[k*np + j] = a1[kk*2 + jj];
				}				
			}
			//b = [b; b1*d]; 
			//c = [d1*c c1];
			for (int k = 0; k<c_a + 2; k++)
			{
				c[k] = 0;
			}			
			c[c_a+1] = 1;
			(*d) = d1*(*d);
			ma1 += 2;
			na2 = 2;
			c_a += 2;
		}
		if (ma1 == 0)
		{
			//a = [a zeros(ma1,na2); b1*c a1];
			for (int k = 0; k<2; k++)
			{
				for (int j = 0; j<2; j++)
				{
					a[k*np+j] = a1[k*2+j];
				}
			}
			//b = [b; b1*d];
			for (int k = 0; k<2; k++)
			{
				b[k] = b1[k];
			}
			//c = [d1*c c1];
			for (int k = 0; k<2; k++)
			{
				c[k] = c1[k];
			}
			(*d) = d1*(*d);
			ma1 += 2;
			na2 = 2;
			c_a += 2;
		}
	}

	for (int i = 0; i<np; i++)
	{
		c[i] *= k0;
	}
	(*d) = k0*(*d);	
}

/*************************************************************************
* @brief   :  Change cutoff frequency for lowpass analog filter,matlab函数
* @inparam :  n			矩阵A的阶数
*			  a,b
*			  wo
* @outparam:  a,b
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void mylp2lp(int n, double *a, double *b, double wo)
{
	for (int i = 0; i < n*n; i++)
	{
		a[i] = wo * a[i];
	}
	for (int i = 0; i < n; i++)
	{
		b[i] = wo * b[i];
	}
}

/*************************************************************************
* @brief   :  Transform lowpass analog filters to bandpass,matlab函数
* @inparam :  n			矩阵A的阶数
*			  a,b,c,d
*			  wo
*			  bw
* @outparam:  a,b,c,d
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void mylp2bp(int n, double **a, double **b, double **c, double *d, double wo, double bw)
{
	double q = wo / bw;
	double *at = (double *)malloc(sizeof(double)*(2 * n)*(2 * n));
	double *bt = (double *)malloc(sizeof(double)*(2 * n));
	double *ct = (double *)malloc(sizeof(double)*(2 * n));
	double dt = *d;

	for (int i = 0; i < 2*n; i++)
	{
		for (int j = 0; j < 2*n; j++)
		{
			if (i < n && j < n)
				at[i * 2 * n + j] = (*a)[+i*n + j] / q * wo;
			else if (i < n && j >= n)
			{
				if (i == j - n)
					at[i * 2 * n + j] = 1 * wo;
				else
					at[i * 2 * n + j] = 0;
			}
			else if (i >= n && j < n)
			{
				if (i - n == j)
					at[i * 2 * n + j] = -1 * wo;
				else
					at[i * 2 * n + j] = 0;
			}
			else if (i >= n && j >= n)
				at[i * 2 * n + j] = 0;
		}
	}
	bt[0] = (*b)[0] * wo;
	for (int i = 1; i < 2 * n; i++)
	{
		bt[i] = 0;
	}
	for (int i = 0; i < 2 * n; i++)
	{
		if (i < n)
			ct[i] = (*c)[i];
		else
			ct[i] = 0;
	}

	*a = (double*)realloc(*a, (2 * n)*(2 * n) * sizeof(double));
	*b = (double*)realloc(*b, (2 * n) * sizeof(double));
	*c = (double*)realloc(*c, (2 * n) * sizeof(double));
	for (int i = 0; i < 2 * n * 2 * n; i++)
		(*a)[i] = at[i];
	for (int i = 0; i < 2 * n; i++)
	{
		(*b)[i] = bt[i];
		(*c)[i] = ct[i];
	}

	free(at);
	free(bt);
	free(ct);
}

/*************************************************************************
* @brief   :  用于模数转换的双线性变换方法,matlab函数
* @inparam :  n			矩阵A的阶数
*			  a,b,c,d	
*			  fs
* @outparam:  a,b,c,d	
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void mybilinear(int n, double *a, double *b, double *c, double *d, double fs)
{
	double t = 1 / fs;
	double r = sqrt(t);
	double *eye_a = (double *)malloc(n*n * sizeof(double));
	double *t1 = (double *)malloc(n*n * sizeof(double));
	double *t2 = (double *)malloc(n*n * sizeof(double));
	double *t2_rinv = (double *)malloc(n*n * sizeof(double));
	double *ad = (double *)malloc(n*n * sizeof(double));
	double *bd = (double *)malloc(n*n * sizeof(double));
	double *cd = (double *)malloc(n*n * sizeof(double));
	double *dd = (double *)malloc(n*n * sizeof(double));
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			if (i == j)
				eye_a[i*n + j] = 1;
			else
				eye_a[i*n + j] = 0;
			t1[i*n + j] = eye_a[i*n + j] + a[i*n + j] * t / 2;
			t2[i*n + j] = eye_a[i*n + j] - a[i*n + j] * t / 2;
			t2_rinv[i*n + j] = eye_a[i*n + j] - a[i*n + j] * t / 2;
		}
	}
	myrinv(t2_rinv, n);
	mytrmul(t2_rinv, t1, n, n, n, ad);
	mytrmul(t2_rinv, b, n, n, 1, bd);
	mytrmul(c, t2_rinv, 1, n, n, cd);
	mytrmul(cd, b, 1, n, 1, dd);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{			
			bd[i*n + j] = bd[i*n + j] * t / r;
			cd[i*n + j] = cd[i*n + j] * r;
			dd[i*n + j] = dd[i*n + j] * t / 2 + *d;

			a[i*n + j] = ad[i*n + j];
		}
	}
	for (int i = 0; i < n; i++)
	{
		b[i] = bd[i*n];
		c[i] = cd[i];
	}
	*d = dd[0];

	free(eye_a);
	free(t1);
	free(t2); 
	free(t2_rinv);
	free(ad);
	free(bd);
	free(cd);
	free(dd);
}

/*************************************************************************
* @brief   :  一般实矩阵约化为Hessenberg矩阵
* @inparam :  a		存放一般实矩阵A,返回上H矩阵
*			  n		矩阵的阶数
* @outparam:  a
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void myhhbg(double a[], int n)
{
	int i, j, k, u, v;
	double d, t;
	for (k = 1; k <= n - 2; k++)
	{
		d = 0.0;
		for (j = k; j <= n - 1; j++)
		{
			u = j*n + k - 1; t = a[u];
			if (fabs(t)>fabs(d))
			{
				d = t; i = j;
			}
		}
		if (fabs(d) + 1.0 != 1.0)
		{
			if (i != k)
			{
				for (j = k - 1; j <= n - 1; j++)
				{
					u = i*n + j; v = k*n + j;
					t = a[u]; a[u] = a[v]; a[v] = t;
				}
				for (j = 0; j <= n - 1; j++)
				{
					u = j*n + i; v = j*n + k;
					t = a[u]; a[u] = a[v]; a[v] = t;
				}
			}
			for (i = k + 1; i <= n - 1; i++)
			{
				u = i*n + k - 1; t = a[u] / d; a[u] = 0.0;
				for (j = k; j <= n - 1; j++)
				{
					v = i*n + j;
					a[v] = a[v] - t*a[k*n + j];
				}
				for (j = 0; j <= n - 1; j++)
				{
					v = j*n + k;
					a[v] = a[v] + t*a[j*n + i];
				}
			}
		}
	}
	return;
}

/*************************************************************************
* @brief   :  用带原点位移的双重步QR方法计算实上H矩阵的全部特征值
* @inparam :  a		存放上H矩阵A
*			  n		上H矩阵A的阶数
*			  eps	控制精度要求
*			  jt	控制最大迭代次数
* @outparam:  u		返回n个特征值的实部
*			  v		返回n个特征值的虚部
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static int myhhqr(double a[], int n, double eps, int jt, double *u, double *v)
{
	int m, it, i, j, k, l, ii, jj, kk, ll;
	double b, c, w, g, xy, p, q, r, x, s, e, f, z, y;
	it = 0; m = n;
	while (m != 0)
	{
		l = m - 1;
		while ((l>0) && (fabs(a[l*n + l - 1])>eps*(fabs(a[(l - 1)*n + l - 1]) + fabs(a[l*n + l])))) l = l - 1;
		ii = (m - 1)*n + m - 1; jj = (m - 1)*n + m - 2;
		kk = (m - 2)*n + m - 1; ll = (m - 2)*n + m - 2;
		if (l == m - 1)
		{
			u[m - 1] = a[(m - 1)*n + m - 1]; v[m - 1] = 0.0;
			m = m - 1; it = 0;
		}
		else if (l == m - 2)
		{
			b = -(a[ii] + a[ll]);
			c = a[ii] * a[ll] - a[jj] * a[kk];
			w = b*b - 4.0*c;
			y = sqrt(fabs(w));
			if (w>0.0)
			{
				xy = 1.0;
				if (b<0.0) xy = -1.0;
				u[m - 1] = (-b - xy*y) / 2.0;
				u[m - 2] = c / u[m - 1];
				v[m - 1] = 0.0; v[m - 2] = 0.0;
			}
			else
			{
				u[m - 1] = -b / 2.0; u[m - 2] = u[m - 1];
				v[m - 1] = y / 2.0; v[m - 2] = -v[m - 1];
			}
			m = m - 2; it = 0;
		}
		else
		{
			if (it >= jt)
			{
				return(-1);
			}
			it = it + 1;
			for (j = l + 2; j <= m - 1; j++)
				a[j*n + j - 2] = 0.0;
			for (j = l + 3; j <= m - 1; j++)
				a[j*n + j - 3] = 0.0;
			for (k = l; k <= m - 2; k++)
			{
				if (k != l)
				{
					p = a[k*n + k - 1]; q = a[(k + 1)*n + k - 1];
					r = 0.0;
					if (k != m - 2) r = a[(k + 2)*n + k - 1];
				}
				else
				{
					x = a[ii] + a[ll];
					y = a[ll] * a[ii] - a[kk] * a[jj];
					ii = l*n + l; jj = l*n + l + 1;
					kk = (l + 1)*n + l; ll = (l + 1)*n + l + 1;
					p = a[ii] * (a[ii] - x) + a[jj] * a[kk] + y;
					q = a[kk] * (a[ii] + a[ll] - x);
					r = a[kk] * a[(l + 2)*n + l + 1];
				}
				if ((fabs(p) + fabs(q) + fabs(r)) != 0.0)
				{
					xy = 1.0;
					if (p<0.0) xy = -1.0;
					s = xy*sqrt(p*p + q*q + r*r);
					if (k != l) a[k*n + k - 1] = -s;
					e = -q / s; f = -r / s; x = -p / s;
					y = -x - f*r / (p + s);
					g = e*r / (p + s);
					z = -x - e*q / (p + s);
					for (j = k; j <= m - 1; j++)
					{
						ii = k*n + j; jj = (k + 1)*n + j;
						p = x*a[ii] + e*a[jj];
						q = e*a[ii] + y*a[jj];
						r = f*a[ii] + g*a[jj];
						if (k != m - 2)
						{
							kk = (k + 2)*n + j;
							p = p + f*a[kk];
							q = q + g*a[kk];
							r = r + z*a[kk]; a[kk] = r;
						}
						a[jj] = q; a[ii] = p;
					}
					j = k + 3;
					if (j >= m - 1) j = m - 1;
					for (i = l; i <= j; i++)
					{
						ii = i*n + k; jj = i*n + k + 1;
						p = x*a[ii] + e*a[jj];
						q = e*a[ii] + y*a[jj];
						r = f*a[ii] + g*a[jj];
						if (k != m - 2)
						{
							kk = i*n + k + 2;
							p = p + f*a[kk];
							q = q + g*a[kk];
							r = r + z*a[kk]; a[kk] = r;
						}
						a[jj] = q; a[ii] = p;
					}
				}
			}
		}
	}
	return(1);
}

/*************************************************************************
* @brief   :  复数除法
* @inparam :  a,b	表示复数a+jb
*			  c,d	表示复数c+jd
* @outparam:  *e,*f	指向返回的复数商 e+jf = (a+jb) / (c+jd)
* @author  :  zone53
* @date    :  2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
static void mycdiv(double a, double b, double c, double d, double *e, double *f)
{
	double p, q, s, w;
	p = a*c; q = -b*d; s = (a + b)*(c - d);
	w = c*c + d*d;
	if (w + 1.0 == 1.0)
	{
		*e = 1.0e+35*a / fabs(a);
		*f = 1.0e+35*b / fabs(b);
	}
	else
	{
		*e = (p - q) / w; *f = (s - p - q) / w;
	}
	return;
}


/*************************************************************************
* @brief   :  求巴特沃斯滤波器系数 matlab对应函数 butter,精度大约在小数点后10~16位
* @inparam :  n		滤波器阶数
*			  Wn[2]	Wn在[0.0, 1.0]之间,与1/2采样率对应
*			  type	1 = "low" 		低通滤波器
*					2 = "bandpass" 	带通滤波器
*					3 = "high" 		高通滤波器		注:没写
*					4 = "stop" 		带阻滤波器		注:没写
*			  analog	0 = digital
*						1 = analog
* @outparam:  ab	长度为 n+1
*			  bb	长度为 n+1 或 2n+1(带通)
* @author  :  zone53
* @date    : 2017/10/18 15:40
* @version :  ver 1.0
*************************************************************************/
void BtwFilter::mybutter(int n, double Wn[], int type, int analog, double* ab, double* bb)
{
	double fs = 2;
	double u[2] = { 0.0, 0.0 };
	//step 1: get analog, pre-warped frequencies
	if (!analog)
	{
		if (type == 1 || type == 3)
		{
			fs = 2;
			double t1=pi*Wn[0] / fs;
			double t2=tan(t1);
			u[0] = 2 * fs*t2;
		}
		else
		{
			fs = 2;
			u[0] = 2 * fs*tan(pi*Wn[0] / fs);
			u[1] = 2 * fs*tan(pi*Wn[1] / fs);
		}
	}
	else if (type == 2 || type == 4)
	{
		if (type == 1 || type == 3)
		{
			u[0] = Wn[0];
		}
		else
		{
			u[1] = Wn[1];
		}
	}

	//step 2: convert to low-pass prototype estimate
	double Bw = 0.0;
	if (type == 1 || type == 3)
	{
		Wn = u;
	}
	else if (type == 2 || type == 4)
	{
		Bw = u[1] - u[0];
		Wn[0] = sqrt(u[0] * u[1]);		//center 
		Wn[1] = 0.0;
	}

	//step 3: Get N-th order Butterworth analog lowpass prototype
	_C_double_complex* p = (_C_double_complex*)malloc(n * sizeof(_C_double_complex));
	double k = 0;
	mybuttap(n, p, &k);

	//Transform to state-space
	int a_size = n;
	double *a = (double *)malloc(sizeof(double) * n * n);
	double *b = (double *)malloc(sizeof(double) * n);
	double *c = (double *)malloc(sizeof(double) * n);
	double d;
	
	myzp2ss(p, n, k, a, b, c, &d);

	if (type == 1)						// Lowpass
		mylp2lp(n, a, b, Wn[0]);
	else if (type == 2)					// Bandpass
	{
		mylp2bp(n, &a, &b, &c, &d, Wn[0], Bw);
		a_size = 2 * n;
	}
	else
		return;

	if (!analog)
	{
		mybilinear(a_size, a, b, c, &d, fs);
	}

	myhhbg(a, a_size);
	double *u_real = (double *)malloc(sizeof(double) *a_size);
	double *v_imag = (double *)malloc(sizeof(double) *a_size);
	double eps = 0.000000000000000000000000000001;
	int jt = 60;
	myhhqr(a, a_size, eps, jt, u_real, v_imag);

	_C_double_complex* p1 = (_C_double_complex*)malloc(a_size * sizeof(_C_double_complex));
	_C_double_complex* ctemp = (_C_double_complex*)malloc((a_size +1) * sizeof(_C_double_complex));
	
	for (int i = 0; i < a_size; i++)
	{
		p1[i]._Val[0] = u_real[i];
		p1[i]._Val[1] = v_imag[i];
	}

	mypoly(p1, a_size, ctemp);

	for (int j = 0; j < a_size + 1; j++)
	{
		ab[j] = creal(ctemp[j]);
	}

	int r_lenth = 0;
	if (type == 1) r_lenth = n;
	else if (type == 2) r_lenth = n * 2;
	else if (type == 3) r_lenth = n;
	else if (type == 4) r_lenth = n;			//这里大小不清楚,待定
	_C_double_complex *r = (_C_double_complex *)malloc(sizeof(_C_double_complex) * r_lenth);
	double w = 0.0;
	Wn[0] = 2 * atan2(Wn[0], 4);
	switch (type)
	{
	case 1:
		for (int i = 0; i < r_lenth; i++)
		{
			r[i]._Val[0] = -1;
			r[i]._Val[1] = 0;
		}
		w = 0;
		break;
	case 2:
		for (int i = 0; i < r_lenth; i++)
		{
			if (i < n)
			{
				r[i]._Val[0] = 1;
				r[i]._Val[1] = 0;
			}
			else
			{
				r[i]._Val[0] = -1;
				r[i]._Val[1] = 0;
			}
		}
		w = Wn[0];
		break;
	case 3:
		for (int i = 0; i < r_lenth; i++)
		{
			r[i]._Val[0] = 1;
			r[i]._Val[1] = 0;
		}
		w = pi;
		break;
	default:
		return;
		break;
	}

	_C_double_complex *r_temp = (_C_double_complex *)malloc(sizeof(_C_double_complex) * (r_lenth+1));
	_C_double_complex *kern = (_C_double_complex *)malloc(sizeof(_C_double_complex) * (r_lenth + 1));
	mypoly(r, r_lenth, r_temp);
	
	for (int j = 0; j < r_lenth + 1; j++)
	{
		bb[j] = creal(r_temp[j]);
	}

	_C_double_complex temp;
	for (int i = 0; i < r_lenth + 1; i++)
	{
		temp._Val[0] = 0;
		temp._Val[1] = -1 * w * i;
		kern[i] = tcexp(temp);
	}

	_C_double_complex c_temp1, c_temp2, c_temp3;
	c_temp3._Val[0] = 0;
	c_temp3._Val[1] = 0;
	for (int i = 0; i < r_lenth + 1; i++)
	{
		c_temp1 = _Cmulcr(kern[i], ab[i]);
		c_temp3._Val[0] += c_temp1._Val[0];
		c_temp3._Val[1] += c_temp1._Val[1];
	}

	c_temp2._Val[0] = 0;
	c_temp2._Val[1] = 0;
	for (int i = 0; i < r_lenth + 1; i++)
	{
		r_temp[i] = _Cmulcr(c_temp3, bb[i]);

		c_temp1 = _Cmulcr(kern[i], bb[i]);
		c_temp2._Val[0] += c_temp1._Val[0];
		c_temp2._Val[1] += c_temp1._Val[1];
	}
	for (int i = 0; i < r_lenth + 1; i++)
	{
		mycdiv(r_temp[i]._Val[0], r_temp[i]._Val[1], c_temp2._Val[0], c_temp2._Val[1], &c_temp1._Val[0], &c_temp1._Val[1]);
		bb[i] = creal(c_temp1);
	}
	free(p);
	free(a);
	free(b);
	free(c);
	free(u_real);
	free(v_imag);
	free(p1);
	free(ctemp);
	free(r);
	free(r_temp);
	free(kern);
}



}