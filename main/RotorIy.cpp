// Copyright (C) 2018 kaz Kojima
//
// This file is part of RotorIy program.  This program is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program;
// see the file COPYING.

#include <cstdint>
#include <cstdio>
//#include <cmath>

#include "RotorIy.h"

/*
  This is an experimental standalone implementation of a 3D rotor
  idendification with geometric algebra.  GA computations are
  hand-compiled to a few arithmetics of Euclidean vector and bivector.

  Some common naming rule for variables:
  Postfix x (resp. y, z) corresponds the standard 3D Euclidean base
  e1 (resp. e2, e3) which is a unit vector on x(resp. y, z)-axis.
  Postfix i (resp. j, k) represents the unit bivector e2^e3 (resp.
  e3^e1, e1^e2), i.e. can be identified with the pure quatanion i
  (resp. j, k).
*/

/*
  RotorIy: Base class.
*/

RotorIy::RotorIy ()
{
  m_S0 = 1.0f;
  m_Si = m_Sj = m_Sk = 0.0f;
}

RotorIy::~RotorIy ()
{
}

void
RotorIy::Show (void)
{
  printf ("S=%2.6f %2.6f %2.6f %2.6f\n", m_S0, m_Si, m_Sj, m_Sk);
}

/*
  Common additional fast finite float math functions.
*/

extern "C" float Sinf(float x);
extern "C" float Cosf(float x);
#define sinf Sinf
#define cosf Cosf

static const float float_ep = 1.192093e-07;
static const float root_ep = 3.452670e-04;
static const float root_root_ep = 1.858136e-02;
static const float float_pi = 3.14159265359;

static float
invSqrt (float x)
{
  union { int32_t i; float f; } u;
  float halfx = 0.5f * x;
  float y = x;
  u.f = y;
  u.i = 0x5f3759df - (u.i>>1);
  y = u.f;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

static float
Sqrt (float x)
{
  return __builtin_sqrtf (x);
}

static float
Sincf(float x)
{
  if (x < 0)
    x = -x;
  if (x >= root_root_ep)
    return sinf(x)/x;
  // Approx. near by 0
  float sc = 1.0;
  if (x >= float_ep)
    {
      float x2 = x*x;
      sc -= x2/6;
      if (x >= root_ep)
	sc += x2*x2/120;
    }

  return sc;
}
  
static float
Atan2 (float y, float x)
{
  float absx = x >= 0 ? x : -x;
  float absy = y >= 0 ? y : -y;
  float absmin = absx >= absy ? absy : absx;
  float absmax = absx >= absy ? absx : absy;
  float a = absmin/absmax;
  float s = a*a;
  // 7th order Taylor approximation
  float r = ((-0.0464964749f*s + 0.15931422f)*s - 0.327622764f)*s*a + a;
  if (absy > absx)
    r = float_pi/2 - r;
  if (x < 0)
    r = float_pi - r;
  if (y < 0)
    r = -r;
  return r;
}

// constants
static const float sq_gravity_mss = (9.80665f*9.80665f);
static const float rotor_ep = 1.0e-6;

/*
  Fontijne-Dost algorism of 3D rotor reconstruction
*/

// Cross product, or (X^Y)*.
inline static void
crossm (float x0, float x1, float x2, float y0, float y1, float y2,
	float& vi, float& vj, float& vk)
{
  vi = x1*y2 - x2*y1;
  vj = x2*y0 - x0*y2;
  vk = x0*y1 - x1*y0;
}

inline static void
ReconstructRotor0 (float x0, float x1, float x2,
		   float y0, float y1, float y2,
		   float xp0, float xp1, float xp2,
		   float yp0, float yp1, float yp2,
		   float& v0, float& vi, float& vj, float& vk)
{
  // (yp + y).(xp - x) + (yp - y)^(xp - x)
  v0 = (yp0 + y0)*(xp0 - x0) + (yp1 + y1)*(xp1 - x1) + (yp2 + y2)*(xp2 - x2);
  crossm (yp0 - y0, yp1 - y1, yp2 - y2, xp0 - x0, xp1 - x1, xp2 - x2,
	  vi, vj, vk);
}

static void
ReconstructRotor (float x0, float x1, float x2,
		  float y0, float y1, float y2,
		  float xp0, float xp1, float xp2,
		  float yp0, float yp1, float yp2,
		  float& v0, float& vi, float& vj, float& vk)
{
  // V = (yp + y).(xp - x) + (yp - y)^(xp - x)
  ReconstructRotor0 (x0, x1, x2, y0, y1, y2,
		     xp0, xp1, xp2, yp0, yp1, yp2,
		     v0, vi, vj, vk);
  // if (|V| > epsilon) return V
  float nm = v0*v0 + vi*vi + vj*vj + vk*vk;
  if (nm > rotor_ep)
    return;

  // Singularity fix.
  // z = (x^y)*, zp = (xp^yp)*
  float z0, z1, z2, zp0, zp1, zp2;
  crossm (x0, x1, x2, y0, y1, y2, z0, z1, z2);
  crossm (xp0, xp1, xp2, yp0, yp1, yp2, zp0, zp1, zp2);
  // Vxz = (zp + z).(xp - x) + (zp - z)^(xp - x)
  ReconstructRotor0 (x0, x1, x2, z0, z1, z2,
		     xp0, xp1, xp2, zp0, zp1, zp2,
		     v0, vi, vj, vk);
  // Vzy = (yp + y).(zp - z) + (yp - y)^(zp - z)
  float u0, ui, uj, uk;
  ReconstructRotor0 (z0, z1, z2, y0, y1, y2,
		     zp0, zp1, zp2, yp0, yp1, yp2,
		     u0, ui, uj, uk);
  // Select larger one.
  float nxz = v0*v0 + vi*vi + vj*vj + vk*vk;
  float nzy = u0*u0 + ui*ui + uj*uj + uk*uk;
  if (nxz > nzy && nxz > rotor_ep)
    return;
  else if (nxz <= nzy && nzy > rotor_ep)
    {
      v0 = u0;
      vi = ui;
      vj = uj;
      vk = uk;
      return;
    }
  // If both norm smaller than epsilon, return 1
  v0 = 1.0f;
  vi = vj = vk = 0.0f;
}

/*
  Ideoms
*/

// v = S*(-e3)/S
// -2 q0 q2 - 2 q1 q3, 2 q0 q1 - 2 q2 q3, -q0^2 + q1^2 + q2^2 - q3^2
inline static void
apply_me3 (float c, float si, float sj, float sk,
	   float& vx, float& vy, float& vz)
{
  vx = -2*c*sj - 2*si*sk;
  vy =  2*c*si - 2*sj*sk;
  vz = -c*c + si*si + sj*sj - sk*sk;
}

// u = S*(e2)/S
// 2 q1 q2 - 2 q0 q3, q0^2 - q1^2 + q2^2 - q3^2,  2 q0 q1 + 2 q2 q3
inline static void
apply_e2 (float c, float si, float sj, float sk,
	   float& ux, float& uy, float& uz)
{
  ux = 2*si*sj - 2*c*sk;
  uy = c*c - si*si + sj*sj - sk*sk;
  uz = 2*c*si + 2*sj*sk;
}

/*
  RotorIyVS: Implementation with versor(rotor).
*/

RotorIyVS::RotorIyVS ()
{
  m_Ii = m_Ij = m_Ik = 0.0f;
  m_gain = 0.0f;
}

RotorIyVS::RotorIyVS (float gain, float dt, float epsilon):
  RotorIy(), m_gain(gain), m_dt(dt), m_epsilon(epsilon)
{
  m_Ii = m_Ij = m_Ik = 0.0f;
}

RotorIyVS::~RotorIyVS ()
{
}

void
RotorIyVS::UpdateIMU (float gi, float gj, float gk,
		      float ax, float ay, float az,
		      float& c, float& si, float& sj, float& sk)
{
  // Wash out gyro drifts
  m_Ii = (1 - m_epsilon)*m_Ii + m_epsilon*gi;
  m_Ij = (1 - m_epsilon)*m_Ij + m_epsilon*gj;
  m_Ik = (1 - m_epsilon)*m_Ik + m_epsilon*gk;
  float omegai = gi - m_Ii;
  float omegaj = gj - m_Ij;
  float omegak = gk - m_Ik;

  if (m_gain > 0)
    {
      // v = S*(-e3)/S;
      float vx, vy, vz;
      apply_me3 (m_S0, m_Si, m_Sj, m_Sk, vx, vy, vz);
      float nm2 = (ax+vx)*(ax+vx)+(ay+vy)*(ay+vy)+(az+vz)*(az+vz);
      float nm3 = ax*ax + ay*ay + az*az;
      // Don't fuse if y+v is too short or if |y| is far from |G|.
      if (nm2 > m_norm_threshold
	  && 0.8f*sq_gravity_mss < nm3 && nm3 < 1.2f*sq_gravity_mss)
	{
	  // u = (1.0/nm)*(y+v);
	  float inm = invSqrt (nm2);
	  float ux = inm*(ax+vx);
	  float uy = inm*(ay+vy);
	  float uz = inm*(az+vz);
	  // P = u*v;
	  float p0, pi, pj, pk;
	  p0 = ux*vx + uy*vy + uz*vz;
	  pi = uy*vz - uz*vy;
	  pj = uz*vx - ux*vz;
	  pk = ux*vy - uy*vx;
	  //printf("%f %f %f %f\n", p0, pi, pj, pk);
	  // Y = -2.0*log (P);
	  // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
	  nm2 = pi*pi + pj*pj + pk*pk;
	  float nm = Sqrt (nm2);
	  // Skip if nm is too small
	  if (nm > float_ep)
	    {
	      float ac = m_gain*(-2.0f)*invSqrt (nm2)*Atan2 (p0, nm);
	      omegai -= ac*pi;
	      omegaj -= ac*pj;
	      omegak -= ac*pk;
	    }
	}
    }

  float delta = 0.5f*m_dt*Sqrt(omegai*omegai + omegaj*omegaj + omegak*omegak);

  float dc = cosf(delta);
  float dsc = -0.5f*m_dt*Sincf(delta);
  float dsi = dsc*omegai;
  float dsj = dsc*omegaj;
  float dsk = dsc*omegak;
  c = dc*m_S0 - dsi*m_Si - dsj*m_Sj - dsk*m_Sk;
  si = dsi*m_S0 + dc*m_Si - dsk*m_Sj + dsj*m_Sk;
  sj = dsj*m_S0 + dsk*m_Si + dc*m_Sj - dsi*m_Sk;
  sk = dsk*m_S0 - dsj*m_Si + dsi*m_Sj + dc*m_Sk;
  // Memowise the result
  m_S0 = c;
  m_Si = si;
  m_Sj = sj;
  m_Sk = sk;
}

void
RotorIyVS::UpdateIMU (float gi, float gj, float gk,
		      float ax, float ay, float az,
		      float mx, float my, float mz,
		      float& c, float& si, float& sj, float& sk)
{
  float omegai, omegaj, omegak;

  m_Ii = (1 - m_epsilon)*m_Ii + m_epsilon*gi;
  m_Ij = (1 - m_epsilon)*m_Ij + m_epsilon*gj;
  m_Ik = (1 - m_epsilon)*m_Ik + m_epsilon*gk;
  omegai = gi - m_Ii;
  omegaj = gj - m_Ij;
  omegak = gk - m_Ik;

  enum { NOFUSE, FUSE_ACC, FUSE_ACC_MAG, } fuse_type;
  if (m_gain == 0)
    fuse_type = NOFUSE;
  else
    {
      // (va,va)vm - (va,vm)va
      float nma = ax*ax + ay*ay + az*az;
      float cma = ax*mx + ay*my + az*mz;
      float tx = nma*mx - cma*ax;
      float ty = nma*my - cma*ay;
      float tz = nma*mz - cma*az;
      float nmt = tx*tx + ty*ty + tz*tz;
      //printf("%f %f\n", nmt, sq_gravity_mss * 100 * 100);
      if (nmt > sq_gravity_mss * 100 * 100
	  && nma > 0.8f*sq_gravity_mss)
	{
	  float inmt = invSqrt (nmt);
	  // Holizontal north unit vector.
	  mx = inmt*tx;
	  my = inmt*ty;
	  mz = inmt*tz;
	  //printf("H %f %f %f\n", mx, my, mz);
	  float inma = invSqrt (nma);
	  ax = inma*ax;
	  ay = inma*ay;
	  az = inma*az;
	  fuse_type = FUSE_ACC_MAG;
	}
      else
	fuse_type = FUSE_ACC;
    }
  if (fuse_type == FUSE_ACC)
    {
      // v = S*(-e3)/S;
      float vx, vy, vz;
      apply_me3 (m_S0, m_Si, m_Sj, m_Sk, vx, vy, vz);
      float nma = ax*ax + ay*ay + az*az;
      float nm2 = (ax+vx)*(ax+vx) + (ay+vy)*(ay+vy) + (az+vz)*(az+vz);
      // Don't fuse if y+v is too short or if |y| is far from |G|.
      //printf("%f %f %f\n", 0.8*sq_gravity_mss, nma, 1.2*sq_gravity_mss);
      if (nm2 > m_norm_threshold
	  && 0.8f*sq_gravity_mss < nma && nma < 1.2f*sq_gravity_mss)
	{
	  // u = (1.0/nm)*(y+v);
	  float inm = invSqrt (nm2);
	  float ux = inm*(ax+vx);
	  float uy = inm*(ay+vy);
	  float uz = inm*(az+vz);
	  //printf("%f %f %f %f\n", inm, vx, vy, vz);
	  // P = u*v;
	  float p0, pi, pj, pk;
	  p0 = ux*vx + uy*vy + uz*vz;
	  pi = uy*vz - uz*vy;
	  pj = uz*vx - ux*vz;
	  pk = ux*vy - uy*vx;
	  // Y = -2.0*log (P);
	  // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
	  nm2 = pi*pi + pj*pj + pk*pk;
	  float nm = Sqrt (nm2);
	  if (nm > float_ep)
	    {
	      float ac = m_gain*(-2.0f)*invSqrt (nm2)*Atan2 (p0, nm);
	      omegai -= ac*pi;
	      omegaj -= ac*pj;
	      omegak -= ac*pk;
	    }
	}
    }
  else if (fuse_type == FUSE_ACC_MAG)
    {
      // v = S*(-e3)/S;
      float vx, vy, vz;
      apply_me3 (m_S0, m_Si, m_Sj, m_Sk, vx, vy, vz);
      // u = S*(e2)/S
      float ux, uy, uz;
      apply_e2 (m_S0, m_Si, m_Sj, m_Sk, ux, uy, uz);
      // Notice that y(ACC) and m(MAG) are normalized already.
      // Compute a rotor P which satisfies Pv = y and Pu = m
      float p0, pi, pj, pk;
      ReconstructRotor (vx, vy, vz, ux, uy, uz,
			ax, ay, az, mx, my, mz,
			p0, pi, pj, pk);
      //printf("P %f %f %f %f\n", p0, pi, pj, pk);
      // Y = -2.0*log (P);
      // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
      float nm2 = pi*pi + pj*pj + pk*pk;
      float nm = Sqrt (nm2);
      if (nm > float_ep)
	{
	  float ac = m_gain*(-2.0f)*invSqrt (nm2)*Atan2 (p0, nm);
	  omegai -= ac*pi;
	  omegaj -= ac*pj;
	  omegak -= ac*pk;
	}
    }

  float delta = 0.5f*m_dt*Sqrt(omegai*omegai + omegaj*omegaj + omegak*omegak);

  float dc = cosf(delta);
  float dsc = -0.5f*m_dt*Sincf(delta);
  float dsi = dsc*omegai;
  float dsj = dsc*omegaj;
  float dsk = dsc*omegak;
  c = dc*m_S0 - dsi*m_Si - dsj*m_Sj - dsk*m_Sk;
  si = dsi*m_S0 + dc*m_Si - dsk*m_Sj + dsj*m_Sk;
  sj = dsj*m_S0 + dsk*m_Si + dc*m_Sj - dsi*m_Sk;
  sk = dsk*m_S0 - dsj*m_Si + dsi*m_Sj + dc*m_Sk;
  // Memowise the result
  m_S0 = c;
  m_Si = si;
  m_Sj = sj;
  m_Sk = sk;
}

void
RotorIyVS::SetGain (float gain)
{
  if (gain >= 0.0f)
    m_gain = gain;
}

/*
  More additional funcs for bivector version.
*/

// cosf(0.5*alpha)/Sincf(0.5*alpha) - 1
static float
kappa (float x)
{
  if (x < 0.0f)
    x = -x;
  x = 0.5f*x;
  if (x >= root_root_ep)
    return x*cosf(x)/sinf(x) - 1;
  // Approx. near by 0
  float k = 0.0f;
  if (x >= float_ep)
    {
      float x2 = x*x;
      k -= x2/3;
      if (x >= root_ep)
	k -= x2*x2/45;
    }

  return k;
}

// inline version of commutator product for Euclidean bivectors.
// Return (1/2.0) (x*y - y*x) where * is geometric product.
inline static void
comm (const float a, const float b, const float c,
      const float d, const float e, const float f,
      float& x, float& y, float& z)
{
  x = -b*f + c*e;
  y =  a*f - c*d;
  z = -a*e + b*d;
}

inline static float
wrap (float x)
{
  if (x > float_pi)
    return x - 2*float_pi;
  else if (x <= -float_pi)
    return x + 2*float_pi;
  return x;
}

/*
  RotorIyVS: Implementation with bivector.
  Using the 3rd interpolation given by Candy and Lasenby when updating
  bivector.
*/

RotorIyBV::RotorIyBV ()
{
  m_alpha = 0.0f;
  m_Bi = m_Bj = m_Bk = 0.0f;
  m_omega0i = m_omega0j = m_omega0k = 0.0f;
  m_omega1i = m_omega1j = m_omega1k = 0.0f;
  m_Ii = m_Ij = m_Ik = 0.0f;
  m_gain = 0.0f;
}

RotorIyBV::RotorIyBV (float gain, float dt, float epsilon):
  RotorIy(), m_gain(gain), m_dt(dt), m_epsilon(epsilon)
{
  m_alpha = 0.0f;
  m_Bi = m_Bj = m_Bk = 0.0f;
  m_omega0i = m_omega0j = m_omega0k = 0.0f;
  m_omega1i = m_omega1j = m_omega1k = 0.0f;
  m_Ii = m_Ij = m_Ik = 0.0f;
}

RotorIyBV::~RotorIyBV ()
{
}

void
RotorIyBV::UpdateIMU (float gi, float gj, float gk,
		      float ax, float ay, float az,
		      float& c, float& si, float& sj, float& sk)
{
  float omegai, omegaj, omegak;

  m_Ii = (1 - m_epsilon)*m_Ii + m_epsilon*gi;
  m_Ij = (1 - m_epsilon)*m_Ij + m_epsilon*gj;
  m_Ik = (1 - m_epsilon)*m_Ik + m_epsilon*gk;
  omegai = gi - m_Ii;
  omegaj = gj - m_Ij;
  omegak = gk - m_Ik;

  if (m_gain > 0)
    {
      // v = S*(-e3)/S;
      float vx, vy, vz;
      apply_me3 (m_S0, m_Si, m_Sj, m_Sk, vx, vy, vz);
      float nm2 = (ax+vx)*(ax+vx) + (ay+vy)*(ay+vy) + (az+vz)*(az+vz);
      float nm3 = ax*ax + ay*ay + az*az;
      // Don't fuse if y+v is too short or if |y| is far from |G|.
      //printf("%f %f %f\n", 0.8*sq_gravity_mss, nm3, 1.2*sq_gravity_mss);
      if (nm2 > m_norm_threshold
	  && 0.8f*sq_gravity_mss < nm3 && nm3 < 1.2f*sq_gravity_mss)
	{
	  // u = (1.0/nm)*(y+v);
	  float inm = invSqrt (nm2);
	  float ux = inm*(ax+vx);
	  float uy = inm*(ay+vy);
	  float uz = inm*(az+vz);
	  // P = u*v;
	  float p0, pi, pj, pk;
	  p0 = ux*vx + uy*vy + uz*vz;
	  pi = uy*vz - uz*vy;
	  pj = uz*vx - ux*vz;
	  pk = ux*vy - uy*vx;
	  // Y = -2.0*log (P);
	  // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
	  nm2 = pi*pi + pj*pj + pk*pk;
	  float nm = Sqrt (nm2);
	  if (nm > float_ep)
	    {
	      float ac = m_gain*(-2.0f)*invSqrt (nm2)*Atan2 (p0, nm);
	      omegai -= ac*pi;
	      omegaj -= ac*pj;
	      omegak -= ac*pk;
	    }
	}
    }

  // 3rd order approximation by Candy and Lasenby.
  // _omega0 = \omega(-T), _omega1 = \omega(0), omega = \omega(T)
  float ci, cj, ck;
  comm (omegai - m_omega0i, omegaj - m_omega0j, omegak - m_omega0k,
	m_omega1i, m_omega1j, m_omega1k,
	ci, cj, ck);

  float dbi, dbj, dbk;
  dbi = ((1.0f/12)*m_dt*(-m_omega0i + 8.0f*m_omega1i + 5.0f*omegai)
	 + (1.0f/24)*m_dt*m_dt*ci);
  dbj = ((1.0f/12)*m_dt*(-m_omega0j + 8.0f*m_omega1j + 5.0f*omegaj)
	 + (1.0f/24)*m_dt*m_dt*cj);
  dbk = ((1.0f/12)*m_dt*(-m_omega0k + 8.0f*m_omega1k + 5.0f*omegak)
	 + (1.0f/24)*m_dt*m_dt*ck);

  // grade 2 part of <B, omega> i.e. commutator [B, omega]
  comm (m_Bi, m_Bj, m_Bk, dbi, dbj, dbk, ci, cj, ck);

  // (cosf(0.5*alpha)/Sincf(0.5*alpha) - 1)[omega + (omega.\phi)\phi]
  float alpha = m_alpha;
  if (alpha > root_ep)
    {
      float k, k2, ip;
      // k = cosf(0.5*alpha)/Sincf(0.5*alpha) - 1
      k = kappa (alpha);
      // k2 = kappa(x)/(x^2) = -1/12 + ? x^2 + ... when x << 1
      k2 = k/(alpha*alpha);
      // m_dt * k * (Omega + (Omega.B)B)
      ip = -(dbi*m_Bi + dbj*m_Bj + dbk*m_Bk)*k2;
      dbi += k*dbi + ip*m_Bi;
      dbj += k*dbj + ip*m_Bj;
      dbk += k*dbk + ip*m_Bk;
    }

  // Updating with the GA version of Bortz equation.
  m_Bi = wrap (m_Bi + dbi - 0.5f*ci);
  m_Bj = wrap (m_Bj + dbj - 0.5f*cj);
  m_Bk = wrap (m_Bk + dbk - 0.5f*ck);

  m_alpha = Sqrt(m_Bi*m_Bi + m_Bj*m_Bj + m_Bk*m_Bk);

  m_omega0i = m_omega1i;
  m_omega0j = m_omega1j;
  m_omega0k = m_omega1k;
  m_omega1i = omegai;
  m_omega1j = omegaj;
  m_omega1k = omegak;

  // S = exp(-0.5*B)
  float phi = -0.5f*m_alpha;
  c = cosf(phi);
  float sc = -0.5f*Sincf(phi);
  si = sc*m_Bi;
  sj = sc*m_Bj;
  sk = sc*m_Bk;
  // Memowise the result
  m_S0 = c;
  m_Si = si;
  m_Sj = sj;
  m_Sk = sk;
}

void
RotorIyBV::UpdateIMU (float gi, float gj, float gk,
		      float ax, float ay, float az,
		      float mx, float my, float mz,
		      float& c, float& si, float& sj, float& sk)
{
  float omegai, omegaj, omegak;

  m_Ii = (1 - m_epsilon)*m_Ii + m_epsilon*gi;
  m_Ij = (1 - m_epsilon)*m_Ij + m_epsilon*gj;
  m_Ik = (1 - m_epsilon)*m_Ik + m_epsilon*gk;
  omegai = gi - m_Ii;
  omegaj = gj - m_Ij;
  omegak = gk - m_Ik;

  enum { NOFUSE, FUSE_ACC, FUSE_ACC_MAG, } fuse_type;
  if (m_gain == 0.0f)
    fuse_type = NOFUSE;
  else
    {
      // (va,va)vm - (va,vm)va
      float nma = ax*ax + ay*ay + az*az;
      float cma = ax*mx + ay*my + az*mz;
      float tx = nma*mx - cma*ax;
      float ty = nma*my - cma*ay;
      float tz = nma*mz - cma*az;
      float nmt = tx*tx + ty*ty + tz*tz;
      //printf("%f %f\n", nmt, sq_gravity_mss * 100 * 100);
      if (nmt > sq_gravity_mss * 100 * 100
	  && nma > 0.8f*sq_gravity_mss)
	{
	  float inmt = invSqrt (nmt);
	  // Holizontal north unit vector.
	  mx = inmt*tx;
	  my = inmt*ty;
	  mz = inmt*tz;
	  //printf("H %f %f %f\n", mx, my, mz);
	  float inma = invSqrt (nma);
	  ax = inma*ax;
	  ay = inma*ay;
	  az = inma*az;
	  fuse_type = FUSE_ACC_MAG;
	}
      else
	fuse_type = FUSE_ACC;
    }
  if (fuse_type == FUSE_ACC)
    {
      // v = S*(-e3)/S;
      float vx, vy, vz;
      apply_me3 (m_S0, m_Si, m_Sj, m_Sk, vx, vy, vz);
      float nma = ax*ax + ay*ay + az*az;
      float nm2 = (ax+vx)*(ax+vx) + (ay+vy)*(ay+vy) + (az+vz)*(az+vz);
      // Don't fuse if y+v is too short or if |y| is far from |G|.
      //printf("%f %f %f\n", 0.8*sq_gravity_mss, nma, 1.2*sq_gravity_mss);
      if (nm2 > m_norm_threshold
	  && 0.8f*sq_gravity_mss < nma && nma < 1.2f*sq_gravity_mss)
	{
	  // u = (1.0/nm)*(y+v);
	  float inm = invSqrt (nm2);
	  float ux = inm*(ax+vx);
	  float uy = inm*(ay+vy);
	  float uz = inm*(az+vz);
	  //printf("%f %f %f %f\n", inm, vx, vy, vz);
	  // P = u*v;
	  float p0, pi, pj, pk;
	  p0 = ux*vx + uy*vy + uz*vz;
	  pi = uy*vz - uz*vy;
	  pj = uz*vx - ux*vz;
	  pk = ux*vy - uy*vx;
	  // Y = -2.0*log (P);
	  // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
	  nm2 = pi*pi + pj*pj + pk*pk;
	  float nm = Sqrt (nm2);
	  if (nm > float_ep)
	    {
	      float ac = m_gain*(-2.0f)*invSqrt (nm2)*Atan2 (p0, nm);
	      omegai -= ac*pi;
	      omegaj -= ac*pj;
	      omegak -= ac*pk;
	    }
	}
    }
  else if (fuse_type == FUSE_ACC_MAG)
    {
      // v = S*(-e3)/S;
      float vx, vy, vz;
      apply_me3 (m_S0, m_Si, m_Sj, m_Sk, vx, vy, vz);
      // u = S*(e2)/S;
      float ux, uy, uz;
      apply_e2 (m_S0, m_Si, m_Sj, m_Sk, ux, uy, uz);
      // Notice that y(ACC) and m(MAG) are normalized already.
      // Compute a rotor P which satisfies Pv = y and Pu = m
      float p0, pi, pj, pk;
      ReconstructRotor (vx, vy, vz, ux, uy, uz,
			ax, ay, az, mx, my, mz,
			p0, pi, pj, pk);
      //printf("P %f %f %f %f\n", p0, pi, pj, pk);
      // Y = -2.0*log (P);
      // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
      float nm2 = pi*pi + pj*pj + pk*pk;
      float nm = Sqrt (nm2);
      if (nm > float_ep)
	{
	  float ac = m_gain*(-2.0f)*invSqrt (nm2)*Atan2 (p0, nm);
	  omegai -= ac*pi;
	  omegaj -= ac*pj;
	  omegak -= ac*pk;
	}
    }

  // 3rd order approximation by Candy and Lasenby.
  // _omega0 = \omega(-T), _omega1 = \omega(0), omega = \omega(T)
  float ci, cj, ck;
  comm (omegai - m_omega0i, omegaj - m_omega0j, omegak - m_omega0k,
	m_omega1i, m_omega1j, m_omega1k,
	ci, cj, ck);

  float dbi, dbj, dbk;
  dbi = ((1.0f/12)*m_dt*(-m_omega0i + 8.0f*m_omega1i + 5.0f*omegai)
	 + (1.0f/24)*m_dt*m_dt*ci);
  dbj = ((1.0f/12)*m_dt*(-m_omega0j + 8.0f*m_omega1j + 5.0f*omegaj)
	 + (1.0f/24)*m_dt*m_dt*cj);
  dbk = ((1.0f/12)*m_dt*(-m_omega0k + 8.0f*m_omega1k + 5.0f*omegak)
	 + (1.0f/24)*m_dt*m_dt*ck);

  // grade 2 part of <B, omega> i.e. commutator [B, omega]
  comm (m_Bi, m_Bj, m_Bk, dbi, dbj, dbk, ci, cj, ck);

  // (cosf(0.5*alpha)/Sincf(0.5*alpha) - 1)[omega + (omega.\phi)\phi]
  float alpha = m_alpha;
  if (alpha > root_ep)
    {
      float k, k2, ip;
      // k = cosf(0.5*alpha)/Sincf(0.5*alpha) - 1
      k = kappa (alpha);
      // k2 = kappa(x)/(x^2) = -1/12 + ? x^2 + ... when x << 1
      k2 = k/(alpha*alpha);
      // m_dt * k * (Omega + (Omega.B)B)
      ip = -(dbi*m_Bi + dbj*m_Bj + dbk*m_Bk)*k2;
      dbi += k*dbi + ip*m_Bi;
      dbj += k*dbj + ip*m_Bj;
      dbk += k*dbk + ip*m_Bk;
    }

  // Updating with the GA version of Bortz equation.
  m_Bi = wrap (m_Bi + dbi - 0.5f*ci);
  m_Bj = wrap (m_Bj + dbj - 0.5f*cj);
  m_Bk = wrap (m_Bk + dbk - 0.5f*ck);

  m_alpha = Sqrt(m_Bi*m_Bi + m_Bj*m_Bj + m_Bk*m_Bk);

  m_omega0i = m_omega1i;
  m_omega0j = m_omega1j;
  m_omega0k = m_omega1k;
  m_omega1i = omegai;
  m_omega1j = omegaj;
  m_omega1k = omegak;

  // S = exp(-0.5*B)
  float phi = -0.5f*m_alpha;
  c = cosf(phi);
  float sc = -0.5f*Sincf(phi);
  si = sc*m_Bi;
  sj = sc*m_Bj;
  sk = sc*m_Bk;
  // Memowise the result
  m_S0 = c;
  m_Si = si;
  m_Sj = sj;
  m_Sk = sk;
}

void
RotorIyBV::SetGain (float gain)
{
  if (gain >= 0.0f)
    m_gain = gain;
}

void
RotorIyBV::Show (void)
{
  printf ("S=%2.6f %2.6f %2.6f %2.6f\n", m_S0, m_Si, m_Sj, m_Sk);
  //printf ("alpha=%2.6f B=%2.6f %2.6f %2.6f\n", m_alpha, m_Bi, m_Bj, m_Bk);
}
