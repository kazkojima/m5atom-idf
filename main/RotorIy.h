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

class RotorIy
{
public:
  RotorIy ();
  virtual ~RotorIy ();
  // Update with IMU value and return the current estimation.
  virtual void UpdateIMU (float gi, float gj, float gk,
			  float ax, float ay, float az,
			  float& c, float& si, float& sj, float& sk) = 0;
  // Set gain.
  virtual void SetGain (float gain) = 0;
  virtual void Show ();
protected:
  // Memowised rotor.
  float m_S0, m_Si, m_Sj, m_Sk;
};

// RotorIyVS: Implementation with versor
class RotorIyVS :
  public RotorIy
{
public:
  RotorIyVS ();
  RotorIyVS (float gain, float dt, float epsilon);
  virtual ~RotorIyVS ();
  // Update with IMU value and return the current estimation.
  void UpdateIMU (float gi, float gj, float gk,
		  float ax, float ay, float az,
		  float& c, float& si, float& sj, float& sk);
  void UpdateIMU (float gi, float gj, float gk,
		  float ax, float ay, float az,
		  float mx, float my, float mz,
		  float& c, float& si, float& sj, float& sk);
  // Set gain.
  void SetGain (float gain);
private:
  // Integrated gyro bivector which shows its drift.
  float m_Ii, m_Ij, m_Ik;
  // Fuse gain.
  float m_gain;
  // Constants
  const float m_dt = 1.0e-3;
  const float m_epsilon = 1.0e-6;
  // Is 1.0(~0.1*GRAVITY_MSS) ok?
  const float m_norm_threshold = 1.0;
};

// RotorIyBV: Implementation with bivector
class RotorIyBV :
  public RotorIy
{
public:
  RotorIyBV ();
  RotorIyBV (float gain, float dt, float epsilon);
  virtual ~RotorIyBV ();
  // Update with IMU value and return the current estimation.
  void UpdateIMU (float gi, float gj, float gk,
		  float ax, float ay, float az,
		  float& c, float& si, float& sj, float& sk);
  void UpdateIMU (float gi, float gj, float gk,
		  float ax, float ay, float az,
		  float mx, float my, float mz,
		  float& c, float& si, float& sj, float& sk);
  // Set gain.
  void SetGain (float gain);
  void Show ();
private:
  // Estimated bivector and its size.
  float m_Bi, m_Bj, m_Bk;
  float m_alpha;
  // Memowised omegas.
  float m_omega0i, m_omega0j, m_omega0k;
  float m_omega1i, m_omega1j, m_omega1k;
  // Integrated gyro bivector which shows its drift.
  float m_Ii, m_Ij, m_Ik;
  // Fuse gain.
  float m_gain;
  // Constants.
  float m_dt = 1.0e-3;
  float m_epsilon = 1.0e-6;
  // Is 1.0(~0.1*GRAVITY_MSS) ok?
  const float m_norm_threshold = 1.0;
};
