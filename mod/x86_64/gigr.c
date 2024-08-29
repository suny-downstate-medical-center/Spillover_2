/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__gigr
#define _nrn_initial _nrn_initial__gigr
#define nrn_cur _nrn_cur__gigr
#define _nrn_current _nrn_current__gigr
#define nrn_jacob _nrn_jacob__gigr
#define nrn_state _nrn_state__gigr
#define _net_receive _net_receive__gigr 
#define fluxes fluxes__gigr 
#define states states__gigr 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define K2 _p[0]
#define K2_columnindex 0
#define K5 _p[1]
#define K5_columnindex 1
#define KX _p[2]
#define KX_columnindex 2
#define KY _p[3]
#define KY_columnindex 3
#define KZ _p[4]
#define KZ_columnindex 4
#define kc _p[5]
#define kc_columnindex 5
#define kf _p[6]
#define kf_columnindex 6
#define eps _p[7]
#define eps_columnindex 7
#define V0 _p[8]
#define V0_columnindex 8
#define V1 _p[9]
#define V1_columnindex 9
#define VM2 _p[10]
#define VM2_columnindex 10
#define VM3 _p[11]
#define VM3_columnindex 11
#define V4 _p[12]
#define V4_columnindex 12
#define VM5 _p[13]
#define VM5_columnindex 13
#define n _p[14]
#define n_columnindex 14
#define r1 _p[15]
#define r1_columnindex 15
#define r2 _p[16]
#define r2_columnindex 16
#define r3 _p[17]
#define r3_columnindex 17
#define ru _p[18]
#define ru_columnindex 18
#define rb _p[19]
#define rb_columnindex 19
#define V _p[20]
#define V_columnindex 20
#define B _p[21]
#define B_columnindex 21
#define Rmax _p[22]
#define Rmax_columnindex 22
#define VmaxA _p[23]
#define VmaxA_columnindex 23
#define VmaxN _p[24]
#define VmaxN_columnindex 24
#define KN _p[25]
#define KN_columnindex 25
#define KA _p[26]
#define KA_columnindex 26
#define x0 _p[27]
#define x0_columnindex 27
#define y0 _p[28]
#define y0_columnindex 28
#define z0 _p[29]
#define z0_columnindex 29
#define R0 _p[30]
#define R0_columnindex 30
#define Rin _p[31]
#define Rin_columnindex 31
#define abg _p[32]
#define abg_columnindex 32
#define x _p[33]
#define x_columnindex 33
#define y _p[34]
#define y_columnindex 34
#define z _p[35]
#define z_columnindex 35
#define R _p[36]
#define R_columnindex 36
#define v2 _p[37]
#define v2_columnindex 37
#define v3 _p[38]
#define v3_columnindex 38
#define v5 _p[39]
#define v5_columnindex 39
#define alpha _p[40]
#define alpha_columnindex 40
#define beta _p[41]
#define beta_columnindex 41
#define gamma _p[42]
#define gamma_columnindex 42
#define Dx _p[43]
#define Dx_columnindex 43
#define Dy _p[44]
#define Dy_columnindex 44
#define Dz _p[45]
#define Dz_columnindex 45
#define DR _p[46]
#define DR_columnindex 46
#define v _p[47]
#define v_columnindex 47
#define _g _p[48]
#define _g_columnindex 48
#define _tsav _p[49]
#define _tsav_columnindex 49
#define _nd_area  *_ppvar[0]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_fluxes(void*);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "fluxes", _hoc_fluxes,
 0, 0
};
 /* declare global and static user variables */
#define Kd Kd_gigr
 double Kd = 0.6;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "Kd_gigr", "uM",
 "K2", "uM",
 "K5", "uM",
 "KX", "uM",
 "KY", "uM",
 "KZ", "uM",
 "kc", "/ms",
 "kf", "/ms",
 "eps", "/ms",
 "V0", "uM/ms",
 "V1", "uM/ms",
 "VM2", "uM/ms",
 "VM3", "uM/ms",
 "V4", "uM/ms",
 "VM5", "uM/ms",
 "r1", "/uM",
 "r2", "/ms",
 "r3", "/ms",
 "ru", "/ms",
 "rb", "/uM^4",
 "V", "uM",
 "B", "uM",
 "Rmax", "uM",
 "VmaxA", "uM/ms",
 "VmaxN", "uM/ms",
 "KN", "uM",
 "KA", "uM",
 0,0
};
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "Kd_gigr", &Kd_gigr,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, _Memb_list*, int);
static void nrn_state(NrnThread*, _Memb_list*, int);
 static void nrn_cur(NrnThread*, _Memb_list*, int);
static void  nrn_jacob(NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, _Memb_list*, int);
static void _ode_matsol(NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[2]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"gigr",
 "K2",
 "K5",
 "KX",
 "KY",
 "KZ",
 "kc",
 "kf",
 "eps",
 "V0",
 "V1",
 "VM2",
 "VM3",
 "V4",
 "VM5",
 "n",
 "r1",
 "r2",
 "r3",
 "ru",
 "rb",
 "V",
 "B",
 "Rmax",
 "VmaxA",
 "VmaxN",
 "KN",
 "KA",
 "x0",
 "y0",
 "z0",
 "R0",
 "Rin",
 0,
 "abg",
 0,
 "x",
 "y",
 "z",
 "R",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 50, _prop);
 	/*initialize range parameters*/
 	K2 = 0.1;
 	K5 = 1;
 	KX = 0.3;
 	KY = 0.2;
 	KZ = 0.1;
 	kc = 0.000166;
 	kf = 1.66e-05;
 	eps = 1.66e-05;
 	V0 = 3.3e-05;
 	V1 = 3.3e-05;
 	VM2 = 0.0001;
 	VM3 = 0.000333;
 	V4 = 4.16e-05;
 	VM5 = 0.0005;
 	n = 10000;
 	r1 = 1;
 	r2 = 0.1;
 	r3 = 4;
 	ru = 0.1;
 	rb = 1e-10;
 	V = 100000;
 	B = 1;
 	Rmax = 13287;
 	VmaxA = 0.51;
 	VmaxN = 1.53;
 	KN = 2;
 	KA = 200;
 	x0 = 0;
 	y0 = 0;
 	z0 = 0;
 	R0 = 0;
 	Rin = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 50;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _net_receive(Point_process*, double*, double);
 static void _thread_mem_init(Datum*);
 static void _thread_cleanup(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _gigr_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 5,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
  _extcall_thread = (Datum*)ecalloc(4, sizeof(Datum));
  _thread_mem_init(_extcall_thread);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 1, _thread_mem_init);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 50, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 gigr /www/projects/Spillover/mod/gigr.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "Glutamate-induced glutamate release (GIGR)";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int fluxes(_threadargsproto_);
 
#define _deriv1_advance _thread[0]._i
#define _dith1 1
#define _recurse _thread[2]._i
#define _newtonspace1 _thread[3]._pvoid
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist2[4];
  static int _slist1[4], _dlist1[4];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset = 0; {
   double _lRv ;
 fluxes ( _threadargs_ ) ;
   Dx = V0 + V1 * ( R / Rmax ) - v2 + v3 + kf * y - kc * x ;
   Dy = v2 - v3 - kf * y ;
   Dz = V4 * ( R / Rmax ) - v5 - eps * z ;
   abg = alpha * pow( x , 4.0 ) / ( beta + gamma * pow( x , 4.0 ) ) ;
   _lRv = abg - VmaxN * R / ( R + KN ) - VmaxA * R / ( R + KA ) ;
   DR = _lRv + Rin ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
 double _lRv ;
 fluxes ( _threadargs_ ) ;
 Dx = Dx  / (1. - dt*( ( - ( kc )*( 1.0 ) ) )) ;
 Dy = Dy  / (1. - dt*( ( - ( kf )*( 1.0 ) ) )) ;
 Dz = Dz  / (1. - dt*( ( - ( eps )*( 1.0 ) ) )) ;
 abg = alpha * pow( x , 4.0 ) / ( beta + gamma * pow( x , 4.0 ) ) ;
 _lRv = abg - VmaxN * R / ( R + KN ) - VmaxA * R / ( R + KA ) ;
 DR = DR  / (1. - dt*( 0.0 )) ;
  return 0;
}
 /*END CVODE*/
 
static int states (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset=0; int error = 0;
 { double* _savstate1 = _thread[_dith1]._pval;
 double* _dlist2 = _thread[_dith1]._pval + 4;
 int _counte = -1;
 if (!_recurse) {
 _recurse = 1;
 {int _id; for(_id=0; _id < 4; _id++) { _savstate1[_id] = _p[_slist1[_id]];}}
 error = nrn_newton_thread(_newtonspace1, 4,_slist2, _p, states, _dlist2, _ppvar, _thread, _nt);
 _recurse = 0; if(error) {abort_run(error);}}
 {
   double _lRv ;
 fluxes ( _threadargs_ ) ;
   Dx = V0 + V1 * ( R / Rmax ) - v2 + v3 + kf * y - kc * x ;
   Dy = v2 - v3 - kf * y ;
   Dz = V4 * ( R / Rmax ) - v5 - eps * z ;
   abg = alpha * pow( x , 4.0 ) / ( beta + gamma * pow( x , 4.0 ) ) ;
   _lRv = abg - VmaxN * R / ( R + KN ) - VmaxA * R / ( R + KA ) ;
   DR = _lRv + Rin ;
   {int _id; for(_id=0; _id < 4; _id++) {
if (_deriv1_advance) {
 _dlist2[++_counte] = _p[_dlist1[_id]] - (_p[_slist1[_id]] - _savstate1[_id])/dt;
 }else{
_dlist2[++_counte] = _p[_slist1[_id]] - _savstate1[_id];}}}
 } }
 return _reset;}
 
static int  fluxes ( _threadargsproto_ ) {
    v2 = VM2 * pow( x , 2.0 ) / ( pow( K2 , 2.0 ) + pow( x , 2.0 ) ) ;
   v3 = VM3 * pow( x , 4.0 ) / ( pow( KX , 4.0 ) + pow( x , 4.0 ) ) * pow( y , 2.0 ) / ( pow( KY , 2.0 ) + pow( y , 2.0 ) ) * pow( z , 4.0 ) / ( pow( KZ , 4.0 ) + pow( z , 4.0 ) ) ;
   v5 = VM5 * z / ( K5 + z ) * pow( x , 2.0 ) / ( pow( Kd , 2.0 ) + pow( x , 2.0 ) ) ;
   alpha = n * r1 * r3 * B * V ;
   beta = ( r2 + r3 ) * ru / rb ;
   gamma = r2 + r3 + r1 * B ;
     return 0; }
 
static double _hoc_fluxes(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 fluxes ( _p, _ppvar, _thread, _nt );
 return(_r);
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{  double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _thread = (Datum*)0; _nt = (NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for general derivimplicit and KINETIC case */
    int __i, __neq = 4;
    double __state = R;
    double __primary_delta = (R + _args[0]) - __state;
    double __dtsav = dt;
    for (__i = 0; __i < __neq; ++__i) {
      _p[_dlist1[__i]] = 0.0;
    }
    _p[_dlist1[3]] = __primary_delta;
    dt *= 0.5;
    v = NODEV(_pnt->node);
#if NRN_VECTORIZED
    _thread = _nt->_ml_list[_mechtype]->_thread;
#endif
    _ode_matsol_instance1(_threadargs_);
    dt = __dtsav;
    for (__i = 0; __i < __neq; ++__i) {
      _p[_slist1[__i]] += _p[_dlist1[__i]];
    }
  } else {
 R = R + _args[0] ;
     }
 } }
 
static int _ode_count(int _type){ return 4;}
 
static void _ode_spec(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 4; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_mem_init(Datum* _thread) {
   _thread[_dith1]._pval = (double*)ecalloc(8, sizeof(double));
   _newtonspace1 = nrn_cons_newtonspace(4);
 }
 
static void _thread_cleanup(Datum* _thread) {
   free((void*)(_thread[_dith1]._pval));
   nrn_destroy_newtonspace(_newtonspace1);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
  int _i; double _save;{
  R = R0;
  x = x0;
  y = y0;
  z = z0;
 {
   x = x0 ;
   y = y0 ;
   z = z0 ;
   R = R0 ;
   }
 
}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt, double _v){double _current=0.;v=_v;{
} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 
}
 
}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {  _deriv1_advance = 1;
 derivimplicit_thread(4, _slist1, _dlist1, _p, states, _ppvar, _thread, _nt);
_deriv1_advance = 0;
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 4; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 } {
   }
}}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = x_columnindex;  _dlist1[0] = Dx_columnindex;
 _slist1[1] = y_columnindex;  _dlist1[1] = Dy_columnindex;
 _slist1[2] = z_columnindex;  _dlist1[2] = Dz_columnindex;
 _slist1[3] = R_columnindex;  _dlist1[3] = DR_columnindex;
 _slist2[0] = R_columnindex;
 _slist2[1] = x_columnindex;
 _slist2[2] = y_columnindex;
 _slist2[3] = z_columnindex;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/www/projects/Spillover/mod/gigr.mod";
static const char* nmodl_file_text = 
  "TITLE Glutamate-induced glutamate release (GIGR)\n"
  "\n"
  "UNITS {\n"
  "    (molar) = (1/liter)\n"
  "    (mM) = (millimolar)\n"
  "    (uM) = (micromolar)\n"
  "}\n"
  "\n"
  "NEURON {\n"
  "    POINT_PROCESS gigr\n"
  "    RANGE K2, K5, KX, KY, KZ, kc, kf, eps, V0, V1, VM2, VM3, V4, VM5\n"
  "    RANGE n, r1, r2, r3, ru, rb, V, B, Rmax, VmaxA, VmaxN, KN, KA, abg\n"
  "    RANGE x0, y0, z0, R0, Rin\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    K2 = 0.1 (uM)\n"
  "    K5 = 1.0 (uM)\n"
  "    Kd = 0.6 (uM)\n"
  "    KX = 0.3 (uM)\n"
  "    KY = 0.2 (uM)\n"
  "    KZ = 0.1 (uM)\n"
  "    kc = 0.166e-3 (/ms)\n"
  "    kf = 0.0166e-3 (/ms)\n"
  "    eps = 0.0166e-3 (/ms)\n"
  "    V0 = 0.033e-3 (uM/ms)\n"
  "    V1 = 0.033e-3 (uM/ms)\n"
  "    VM2 = 0.1e-3 (uM/ms)\n"
  "    VM3 = 0.333e-3 (uM/ms)\n"
  "    V4 = 0.0416e-3 (uM/ms)\n"
  "    VM5 = 0.5e-3 (uM/ms)\n"
  "\n"
  "    n = 10000\n"
  "    r1 = 1000e-3 (/uM /ms)\n"
  "    r2 = 100e-3 (/ms)\n"
  "    r3 = 4000e-3 (/ms)\n"
  "    ru = 100e-3 (/ms)\n"
  "    rb = 1e-10 (/uM^4 /ms)\n"
  "    V = 100000 (uM)\n"
  "    B = 1 (uM)\n"
  "    Rmax = 13287 (uM)\n"
  "    VmaxA = 510e-3 (uM/ms)\n"
  "    VmaxN = 1530e-3 (uM/ms)\n"
  "    KN = 2 (uM)\n"
  "    KA = 200 (uM)\n"
  "\n"
  "    x0 = 0\n"
  "    y0 = 0\n"
  "    z0 = 0\n"
  "    R0 = 0\n"
  "    Rin = 0\n"
  "} \n"
  "\n"
  "ASSIGNED { \n"
  "    v2\n"
  "    v3\n"
  "    v5\n"
  "    alpha\n"
  "    beta \n"
  "    gamma\n"
  "    abg\n"
  "}\n"
  "\n"
  "STATE { x y z R }\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE states METHOD derivimplicit\n"
  "}\n"
  "\n"
  "INITIAL {  \n"
  "    x = x0\n"
  "    y = y0\n"
  "    z = z0\n"
  "    R = R0\n"
  "}\n"
  "\n"
  "DERIVATIVE states { \n"
  "    LOCAL Rv\n"
  "    fluxes()\n"
  "    x' = V0 + V1*(R/Rmax) - v2 + v3 + kf*y -kc*x\n"
  "    y' = v2 - v3 - kf*y\n"
  "    z' = V4*(R/Rmax) - v5 - eps*z\n"
  "    abg = alpha*x^4/(beta + gamma*x^4) \n"
  "    Rv =  abg - VmaxN*R/(R + KN) - VmaxA*R/(R + KA)\n"
  "    R' = Rv + Rin\n"
  "}\n"
  "\n"
  "PROCEDURE fluxes() {\n"
  "    UNITSOFF\n"
  "    v2 = VM2 * x^2/(K2^2 + x^2)\n"
  "    v3 = VM3 * x^4/(KX^4 + x^4) * y^2/(KY^2 + y^2) * z^4/(KZ^4 + z^4)\n"
  "    v5 = VM5 * z/(K5 + z) * x^2/(Kd^2 + x^2)\n"
  "\n"
  "    alpha = n*r1*r3*B*V\n"
  "    beta = (r2 + r3)*ru/rb\n"
  "    gamma = r2 + r3 + r1*B \n"
  "    UNITSON\n"
  "}\n"
  "\n"
  "NET_RECEIVE( rin ) {\n"
  "    R = R + rin\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "Glutamate-induced dlutamate release mechanism taken from [1].\n"
  "\n"
  "[1] Larter R. and Glendening Craig M., Glutamate-induced glutamate release: A proposed mechanismfor calcium bursting in astrocytes, : Chaos: An Interdisciplinary Journal of Nonlinear Science 15, 047511 (2005); doi: 10.1063/1.2102467\n"
  "\n"
  "ENDCOMMENT\n"
  ;
#endif
