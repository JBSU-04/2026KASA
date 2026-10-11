#include "new_lane/cubic_spline_planner.hpp"
#include <vector>
template <class T>
ostream &operator<<(ostream &os, vector<T> V)
{
	os << "[ ";
	for (auto v : V)
		os << v << " ";
	return os << "]";
}
// #define trace(...) __f(#__VA_ARGS__, __VA_ARGS__)
// template <typename Arg1>
// void __f(const char* name, Arg1&& arg1){cerr << name << " : " << arg1 << endl;}
// template <typename Arg1, typename... Args>
// void __f(const char* names, Arg1&& arg1, Args&&... args){ const char* comma = strchr(names + 1,
// ','); cerr.write(names, comma - names) << " : " << arg1<<" | "; __f(comma+1, args...); }
#define trace(...) 42
// Python NONE equivalents :
// single variable -1e9
// Splines: http://www.cs.cornell.edu/courses/cs4620/2013fa/lectures/16spline-curves.pdf

/************************
For a vector of points (x_i, y_i), it generates cubic splines 
between each (x_i, y_i) and (x_i+1, y_i+1) as :

f_i(x) = y_i(== a_i) + b_i*(x - x_i) + c_i*(x - x_i)^2 + d_i*(x - x_i)^3

where h_i = x_i - x_i-1
************************/

void printVecD(vecD A)
{
	cout << "\n------------Printing the vector----------\n";
	for (unsigned int i = 0; i < A.size(); i++)
		cout << A[i] << "\n";
	cout << "\n-----------------------------------------\n"
		 << endl;
}

MatrixXd Spline::calc_A(vecD h) // get matrix A
{
	MatrixXd A = MatrixXd::Zero(nx, nx);
	A(0, 0) = 1.0;
	for (int i = 0; i < nx - 1; i++)
	{
		if (i != nx - 2)
			A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
		A(i + 1, i) = h[i];
		A(i, i + 1) = h[i];
	}

	A(0, 1) = 0.0;
	A(nx - 1, nx - 2) = 0.0;
	A(nx - 1, nx - 1) = 1.0;

	return A;
}

MatrixXd Spline::calc_B(vecD h) // get matrix B
{
	MatrixXd B = MatrixXd::Zero(nx, 1);

	for (int i = 0; i < nx - 2; i++)
		B(i + 1, 0) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];

	return B;
}

int Spline::search_index(double p) // returns the index of the just greater element
{
	auto it = upper_bound(x.begin(), x.end(), p);
	if (it == x.end())
	{
		return x.size() - 2;
	}
	else
	{
		return it - x.begin() - 1;
	}
}

void Spline::init(vecD x_in, vecD y_in) // calculates the coeffs for splines
{
	x = x_in;
	y = y_in;
	nx = x.size();
	if (nx < 2)
	{
		return;
	}

	vecD h;
	h.reserve(nx - 1);
	for (int i = 1; i < nx; i++)
		h.push_back(x[i] - x[i - 1]);
	a = y;

	b.clear();
	c.clear();
	d.clear();

	if (nx == 2)
	{
		c.assign(2, 0.0);
		d.push_back(0.0);
		b.push_back((a[1] - a[0]) / h[0]);
		return;
	}

	// Solve tridiagonal system for c using Thomas algorithm: O(N) runtime
	// (Replaces O(N^3) dense matrix inversion which took ~2.8s for 500 points)
	const int m = nx - 2;
	vecD c_prime(m, 0.0);
	vecD d_prime(m, 0.0);

	double b0 = 2.0 * (h[0] + h[1]);
	double d0 = 3.0 * (a[2] - a[1]) / h[1] - 3.0 * (a[1] - a[0]) / h[0];
	c_prime[0] = h[1] / b0;
	d_prime[0] = d0 / b0;

	for (int j = 1; j < m; ++j)
	{
		double aj = h[j];
		double bj = 2.0 * (h[j] + h[j + 1]);
		double dj = 3.0 * (a[j + 2] - a[j + 1]) / h[j + 1] - 3.0 * (a[j + 1] - a[j]) / h[j];
		double denom = bj - aj * c_prime[j - 1];
		if (j < m - 1)
		{
			c_prime[j] = h[j + 1] / denom;
		}
		d_prime[j] = (dj - aj * d_prime[j - 1]) / denom;
	}

	c.assign(nx, 0.0);
	c[nx - 2] = d_prime[m - 1];
	for (int j = m - 2; j >= 0; --j)
	{
		c[j + 1] = d_prime[j] - c_prime[j] * c[j + 2];
	}

	d.reserve(nx - 1);
	b.reserve(nx - 1);
	for (int i = 0; i < nx - 1; i++)
	{
		d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
		double tb = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
		b.push_back(tb);
	}
}

double Spline::calc(double t) // find y at given x
{
	if (x.size() == 0)
	{
		return NONE;
	}
	else if (t < x[0])
	{
		return NONE;
	}
	else if (t > x[nx - 1])
	{
		return NONE;
	}
	int i = search_index(t);
	double dx = t - x[i];
	if (i > nx - 1)
	{
		return NONE;
	}
	// assert(i<b.size());
	double result = a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
	return result;
}

double Spline::calcd(double t) // find y_dot at given x
{
	if (t < x[0])
	{
		return NONE;
	}
	else if (t > x[nx - 1])
	{
		return NONE;
	}
	int i = search_index(t);
	double dx = t - x[i];
	double result = b[i] + 2 * c[i] * dx + 3 * d[i] * dx * dx;

	return result;
}

double Spline::calcdd(double t) // y_doubleDot at given x
{
	if (t < x[0])
		return NONE;
	else if (t > x[nx - 1])
		return NONE;

	int i = search_index(t);

	double dx = t - x[i];
	double result = 2 * c[i] + 6 * d[i] * dx;

	return result;
}

/****************************
Spline 2D generates the parametric cubic spline 
of x and y as a function of s:

x = f(s)
y = g(s)
****************************/

vecD Spline2D::calc_s(vecD x, vecD y) // approximately calculates s along the spline
{
	vecD dx;
	for (unsigned int i = 1; i < x.size(); i++)
	{
		dx.push_back(x[i] - x[i - 1]);
	}
	vecD dy;
	for (unsigned int i = 1; i < x.size(); i++)
	{
		dy.push_back(y[i] - y[i - 1]);
	}
	ds.clear();
	for (unsigned int i = 0; i < dx.size(); i++)
	{
		double temp;
		temp = sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
		ds.push_back(temp);
	}

	vecD t;
	t.push_back(0);

	for (unsigned int i = 0; i < ds.size(); i++)
	{
		t.push_back(t.back() + ds[i]);
	}

	return t;
}

void Spline2D::calc_position(double &x, double &y, double t)
{
	x = sx.calc(t);
	y = sy.calc(t);
}

double Spline2D::calc_curvature(double t)
{
	double dx = sx.calcd(t);
	double ddx = sx.calcdd(t);

	double dy = sy.calcd(t);
	double ddy = sy.calcdd(t);

	// DOUBT : denominator should have power of 3/2.
	// https://www.math24.net/curvature-radius/
	// ==============================================================================
	double k = (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
	// ==============================================================================

	return k;
}

double Spline2D::calc_yaw(double t)
{
	double dx = sx.calcd(t);
	double dy = sy.calcd(t);

	if (dx == 0)
	{
		return 1.57 * (dy > 0);
	}
	double yaw = atan2(dy, dx);

	return yaw;
}

double Spline2D::get_s_last()
{
	return s.back();
}

// generates the Spline2D with points along the spline at distance = ds, also returns
// yaw and curvature
Spline2D calc_spline_course(vecD x, vecD y, vecD &rx, vecD &ry, vecD &ryaw, vecD &rk, double ds)
{

	
	Spline2D sp(x, y);	// DIEEEEEEEEEEEEEEE(32767 roop), char 최대 값 넘어서 죽음(2byte)
	vecD s;
	double sRange = sp.get_s_last();
	double sInc = 0;
	while (1)
	{
		if (sInc >= sRange)
		{
			break;
		}
		s.push_back(sInc);
		sInc = sInc + ds;
	}
	// for(i =0 ; i*ds<sRange;i++)  i*ds = sInc
	//   s[i]=i*ds;
	rx.resize(s.size());
	ry.resize(s.size());
	ryaw.resize(s.size());
	rk.resize(s.size());
	for (int i = 0; i < s.size(); i++)
	{
		double ix, iy;
		sp.calc_position(ix, iy, s[i]);
		rx[i] = ix;
		ry[i] = iy;
		ryaw[i] = sp.calc_yaw(s[i]);
		rk[i] = sp.calc_curvature(s[i]);
	}
	return sp;
}
