/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
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
 
#define nrn_init _nrn_init__adaptive_shom_NMDA_stp
#define _nrn_initial _nrn_initial__adaptive_shom_NMDA_stp
#define nrn_cur _nrn_cur__adaptive_shom_NMDA_stp
#define _nrn_current _nrn_current__adaptive_shom_NMDA_stp
#define nrn_jacob _nrn_jacob__adaptive_shom_NMDA_stp
#define nrn_state _nrn_state__adaptive_shom_NMDA_stp
#define _net_receive _net_receive__adaptive_shom_NMDA_stp 
#define reset_max reset_max__adaptive_shom_NMDA_stp 
#define release release__adaptive_shom_NMDA_stp 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg(int);
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define Cmax _p[0]
#define Cmax_columnindex 0
#define Alpha _p[1]
#define Alpha_columnindex 1
#define Beta _p[2]
#define Beta_columnindex 2
#define Erev _p[3]
#define Erev_columnindex 3
#define mg _p[4]
#define mg_columnindex 4
#define eta _p[5]
#define eta_columnindex 5
#define alpha _p[6]
#define alpha_columnindex 6
#define gmax _p[7]
#define gmax_columnindex 7
#define learning_rate_w_LTP _p[8]
#define learning_rate_w_LTP_columnindex 8
#define learning_rate_w_LTD _p[9]
#define learning_rate_w_LTD_columnindex 9
#define learning_rate_thresh_LTP _p[10]
#define learning_rate_thresh_LTP_columnindex 10
#define learning_rate_thresh_LTD _p[11]
#define learning_rate_thresh_LTD_columnindex 11
#define learning_rate_thresh_KD_LTD _p[12]
#define learning_rate_thresh_KD_LTD_columnindex 12
#define w0 _p[13]
#define w0_columnindex 13
#define ca_nmdai_max _p[14]
#define ca_nmdai_max_columnindex 14
#define cali_max _p[15]
#define cali_max_columnindex 15
#define cati_max _p[16]
#define cati_max_columnindex 16
#define active_syn_flag _p[17]
#define active_syn_flag_columnindex 17
#define nmda_ca_fraction _p[18]
#define nmda_ca_fraction_columnindex 18
#define Cdur _p[19]
#define Cdur_columnindex 19
#define n1 _p[20]
#define n1_columnindex 20
#define n2 _p[21]
#define n2_columnindex 21
#define KD1 _p[22]
#define KD1_columnindex 22
#define KD2 _p[23]
#define KD2_columnindex 23
#define KD_LTD _p[24]
#define KD_LTD_columnindex 24
#define n_LTD _p[25]
#define n_LTD_columnindex 25
#define U _p[26]
#define U_columnindex 26
#define u0 _p[27]
#define u0_columnindex 27
#define iNMDA _p[28]
#define iNMDA_columnindex 28
#define g _p[29]
#define g_columnindex 29
#define last_dopamine _p[30]
#define last_dopamine_columnindex 30
#define weight _p[31]
#define weight_columnindex 31
#define lthresh_LTP _p[32]
#define lthresh_LTP_columnindex 32
#define hthresh_LTP _p[33]
#define hthresh_LTP_columnindex 33
#define lthresh_LTD _p[34]
#define lthresh_LTD_columnindex 34
#define kernel _p[35]
#define kernel_columnindex 35
#define Ron _p[36]
#define Ron_columnindex 36
#define Roff _p[37]
#define Roff_columnindex 37
#define Rinf _p[38]
#define Rinf_columnindex 38
#define Rtau _p[39]
#define Rtau_columnindex 39
#define synon _p[40]
#define synon_columnindex 40
#define B _p[41]
#define B_columnindex 41
#define ica_nmda _p[42]
#define ica_nmda_columnindex 42
#define cati _p[43]
#define cati_columnindex 43
#define cali _p[44]
#define cali_columnindex 44
#define ca_nmdai _p[45]
#define ca_nmdai_columnindex 45
#define x _p[46]
#define x_columnindex 46
#define DRon _p[47]
#define DRon_columnindex 47
#define DRoff _p[48]
#define DRoff_columnindex 48
#define _g _p[49]
#define _g_columnindex 49
#define _tsav _p[50]
#define _tsav_columnindex 50
#define _nd_area  *_ppvar[0]._pval
#define _ion_ca_nmdai	*_ppvar[2]._pval
#define _ion_ica_nmda	*_ppvar[3]._pval
#define _ion_dica_nmdadv	*_ppvar[4]._pval
#define _ion_cali	*_ppvar[5]._pval
#define _ion_cati	*_ppvar[6]._pval
#define dopamine	*_ppvar[7]._pval
#define _p_dopamine	_ppvar[7]._pval
#define stimulus_flag	*_ppvar[8]._pval
#define _p_stimulus_flag	_ppvar[8]._pval
 
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
 static int hoc_nrnpointerindex =  7;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_hthresh(void*);
 static double _hoc_lthresh(void*);
 static double _hoc_min(void*);
 static double _hoc_max(void*);
 static double _hoc_mgblock(void*);
 static double _hoc_quadratic(void*);
 static double _hoc_reset_max(void*);
 static double _hoc_sigmoidal(void*);
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
 _p = _prop->param; _ppvar = _prop->dparam;
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
 "hthresh", _hoc_hthresh,
 "lthresh", _hoc_lthresh,
 "min", _hoc_min,
 "max", _hoc_max,
 "mgblock", _hoc_mgblock,
 "quadratic", _hoc_quadratic,
 "reset_max", _hoc_reset_max,
 "sigmoidal", _hoc_sigmoidal,
 0, 0
};
#define _f_mgblock _f_mgblock_adaptive_shom_NMDA_stp
#define hthresh hthresh_adaptive_shom_NMDA_stp
#define lthresh lthresh_adaptive_shom_NMDA_stp
#define min min_adaptive_shom_NMDA_stp
#define max max_adaptive_shom_NMDA_stp
#define mgblock mgblock_adaptive_shom_NMDA_stp
#define quadratic quadratic_adaptive_shom_NMDA_stp
#define sigmoidal sigmoidal_adaptive_shom_NMDA_stp
 extern double _f_mgblock( double );
 extern double hthresh( double , double , double );
 extern double lthresh( double , double , double );
 extern double min( double , double );
 extern double max( double , double );
 extern double mgblock( double );
 extern double quadratic( double , double , double );
 extern double sigmoidal( double , double , double );
 /* declare global and static user variables */
#define tauF tauF_adaptive_shom_NMDA_stp
 double tauF = 0;
#define tauR tauR_adaptive_shom_NMDA_stp
 double tauR = 100;
#define tau tau_adaptive_shom_NMDA_stp
 double tau = 3;
#define usetable usetable_adaptive_shom_NMDA_stp
 double usetable = 1;
#define wmin wmin_adaptive_shom_NMDA_stp
 double wmin = 0.001;
#define wmax wmax_adaptive_shom_NMDA_stp
 double wmax = 0.006;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "U", 0, 1,
 "usetable_adaptive_shom_NMDA_stp", 0, 1,
 "u0", 0, 1,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "wmax_adaptive_shom_NMDA_stp", "uS",
 "wmin_adaptive_shom_NMDA_stp", "uS",
 "tau_adaptive_shom_NMDA_stp", "ms",
 "tauR_adaptive_shom_NMDA_stp", "ms",
 "tauF_adaptive_shom_NMDA_stp", "ms",
 "Cmax", "mM",
 "Alpha", "/ms",
 "Beta", "/ms",
 "Erev", "mV",
 "mg", "mM",
 "eta", "/mV",
 "alpha", "/mV",
 "gmax", "uS",
 "w0", "uS",
 "U", "1",
 "u0", "1",
 "iNMDA", "nA",
 "g", "uS",
 0,0
};
 static double Roff0 = 0;
 static double Ron0 = 0;
 static double delta_t = 0.01;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "wmax_adaptive_shom_NMDA_stp", &wmax_adaptive_shom_NMDA_stp,
 "wmin_adaptive_shom_NMDA_stp", &wmin_adaptive_shom_NMDA_stp,
 "tau_adaptive_shom_NMDA_stp", &tau_adaptive_shom_NMDA_stp,
 "tauR_adaptive_shom_NMDA_stp", &tauR_adaptive_shom_NMDA_stp,
 "tauF_adaptive_shom_NMDA_stp", &tauF_adaptive_shom_NMDA_stp,
 "usetable_adaptive_shom_NMDA_stp", &usetable_adaptive_shom_NMDA_stp,
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
 
#define _cvode_ieq _ppvar[10]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"adaptive_shom_NMDA_stp",
 "Cmax",
 "Alpha",
 "Beta",
 "Erev",
 "mg",
 "eta",
 "alpha",
 "gmax",
 "learning_rate_w_LTP",
 "learning_rate_w_LTD",
 "learning_rate_thresh_LTP",
 "learning_rate_thresh_LTD",
 "learning_rate_thresh_KD_LTD",
 "w0",
 "ca_nmdai_max",
 "cali_max",
 "cati_max",
 "active_syn_flag",
 "nmda_ca_fraction",
 "Cdur",
 "n1",
 "n2",
 "KD1",
 "KD2",
 "KD_LTD",
 "n_LTD",
 "U",
 "u0",
 0,
 "iNMDA",
 "g",
 "last_dopamine",
 "weight",
 "lthresh_LTP",
 "hthresh_LTP",
 "lthresh_LTD",
 "kernel",
 0,
 "Ron",
 "Roff",
 0,
 "dopamine",
 "stimulus_flag",
 0};
 static Symbol* _ca_nmda_sym;
 static Symbol* _cal_sym;
 static Symbol* _cat_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 51, _prop);
 	/*initialize range parameters*/
 	Cmax = 1;
 	Alpha = 4;
 	Beta = 0.01;
 	Erev = 0;
 	mg = 1;
 	eta = 0.28;
 	alpha = 0.072;
 	gmax = 1;
 	learning_rate_w_LTP = 0.01;
 	learning_rate_w_LTD = 0.01;
 	learning_rate_thresh_LTP = 0.01;
 	learning_rate_thresh_LTD = 0.01;
 	learning_rate_thresh_KD_LTD = 0.01;
 	w0 = 0.00188;
 	ca_nmdai_max = 0;
 	cali_max = 0;
 	cati_max = 0;
 	active_syn_flag = 0;
 	nmda_ca_fraction = 0.15;
 	Cdur = 1.1;
 	n1 = 5;
 	n2 = 5;
 	KD1 = 1;
 	KD2 = 1;
 	KD_LTD = 1;
 	n_LTD = 1;
 	U = 0.3;
 	u0 = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 51;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 11, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_nmda_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2]._pval = &prop_ion->param[1]; /* ca_nmdai */
 	_ppvar[3]._pval = &prop_ion->param[3]; /* ica_nmda */
 	_ppvar[4]._pval = &prop_ion->param[4]; /* _ion_dica_nmdadv */
 prop_ion = need_memb(_cal_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[5]._pval = &prop_ion->param[1]; /* cali */
 prop_ion = need_memb(_cat_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[6]._pval = &prop_ion->param[1]; /* cati */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 
#define _tqitem &(_ppvar[9]._pvoid)
 static void _net_receive(Point_process*, double*, double);
 static void _net_init(Point_process*, double*, double);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _adaptive_shom_NMDA_stp_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca_nmda", 2.0);
 	ion_reg("cal", 2.0);
 	ion_reg("cat", 2.0);
 	_ca_nmda_sym = hoc_lookup("ca_nmda_ion");
 	_cal_sym = hoc_lookup("cal_ion");
 	_cat_sym = hoc_lookup("cat_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 51, 11);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "cat_ion");
  hoc_register_dparam_semantics(_mechtype, 7, "pointer");
  hoc_register_dparam_semantics(_mechtype, 8, "pointer");
  hoc_register_dparam_semantics(_mechtype, 9, "netsend");
  hoc_register_dparam_semantics(_mechtype, 10, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_init[_mechtype] = _net_init;
 pnt_receive_size[_mechtype] = 8;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 adaptive_shom_NMDA_stp /home/shadeform/Spillover_2/mod/adaptive_shom_NMDA_stp.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double *_t_mgblock;
static int _reset;
static char *modelname = "simple NMDA receptors";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int reset_max();
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static double _n_mgblock(double);
 static int _slist1[2], _dlist1[2];
 static int release(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   DRon = ( synon * Rinf - Ron ) / Rtau ;
   DRoff = - Beta * Roff ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 DRon = DRon  / (1. - dt*( ( ( ( - 1.0 ) ) ) / Rtau )) ;
 DRoff = DRoff  / (1. - dt*( ( - Beta )*( 1.0 ) )) ;
  return 0;
}
 /*END CVODE*/
 static int release () {_reset=0;
 {
    Ron = Ron + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / Rtau)))*(- ( ( ( ( synon )*( Rinf ) ) ) / Rtau ) / ( ( ( ( - 1.0 ) ) ) / Rtau ) - Ron) ;
    Roff = Roff + (1. - exp(dt*(( - Beta )*( 1.0 ))))*(- ( 0.0 ) / ( ( - Beta )*( 1.0 ) ) - Roff) ;
   }
  return 0;
}
 static double _mfac_mgblock, _tmin_mgblock;
 static void _check_mgblock();
 static void _check_mgblock() {
  static int _maktable=1; int _i, _j, _ix = 0;
  double _xi, _tmax;
  static double _sav_mg;
  if (!usetable) {return;}
  if (_sav_mg != mg) { _maktable = 1;}
  if (_maktable) { double _x, _dx; _maktable=0;
   _tmin_mgblock =  - 140.0 ;
   _tmax =  80.0 ;
   _dx = (_tmax - _tmin_mgblock)/1000.; _mfac_mgblock = 1./_dx;
   for (_i=0, _x=_tmin_mgblock; _i < 1001; _x += _dx, _i++) {
    _t_mgblock[_i] = _f_mgblock(_x);
   }
   _sav_mg = mg;
  }
 }

 double mgblock(double _lv){ _check_mgblock();
 return _n_mgblock(_lv);
 }

 static double _n_mgblock(double _lv){ int _i, _j;
 double _xi, _theta;
 if (!usetable) {
 return _f_mgblock(_lv); 
}
 _xi = _mfac_mgblock * (_lv - _tmin_mgblock);
 if (isnan(_xi)) {
  return _xi; }
 if (_xi <= 0.) {
 return _t_mgblock[0];
 }
 if (_xi >= 1000.) {
 return _t_mgblock[1000];
 }
 _i = (int) _xi;
 return _t_mgblock[_i] + (_xi - (double)_i)*(_t_mgblock[_i+1] - _t_mgblock[_i]);
 }

 
double _f_mgblock (  double _lv ) {
   double _lmgblock;
 _lmgblock = 1.0 / ( 1.0 + mg * eta * exp ( - alpha * _lv ) ) ;
   
return _lmgblock;
 }
 
static double _hoc_mgblock(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
  _r =  mgblock (  *getarg(1) );
 return(_r);
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{    _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(Object*); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = 0;}
 {
   _args[5] = _args[5] * exp ( - ( t - _args[7] ) / tauR ) ;
   _args[5] = _args[5] + ( _args[4] * ( exp ( - ( t - _args[7] ) / tau ) - exp ( - ( t - _args[7] ) / tauR ) ) / ( tau / tauR - 1.0 ) ) ;
   _args[4] = _args[4] * exp ( - ( t - _args[7] ) / tau ) ;
   x = 1.0 - _args[4] - _args[5] ;
   if ( tauF > 0.0 ) {
     _args[6] = _args[6] * exp ( - ( t - _args[7] ) / tauF ) ;
     _args[6] = _args[6] + U * ( 1.0 - _args[6] ) ;
     }
   else {
     _args[6] = U ;
     }
   if ( _lflag  == 0.0 ) {
     active_syn_flag = 1.0 ;
     _args[2] = _args[2] + 1.0 ;
     if (  ! _args[1] ) {
       _args[3] = _args[3] * exp ( - Beta * ( t - _args[7] ) ) ;
       _args[7] = t ;
       _args[1] = 1.0 ;
       synon = synon + weight * _args[6] ;
           if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = Ron;
    double __primary = (Ron + _args[3] ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( ( ( - 1.0 ) ) ) / Rtau ) ) )*( - ( ( ( ( synon )*( Rinf ) ) ) / Rtau ) / ( ( ( ( - 1.0 ) ) ) / Rtau ) - __primary );
    Ron += __primary;
  } else {
 Ron = Ron + _args[3]  ;
         }
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = Roff;
    double __primary = (Roff - _args[3] ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - Beta )*( 1.0 ) ) ) )*( - ( 0.0 ) / ( ( - Beta )*( 1.0 ) ) - __primary );
    Roff += __primary;
  } else {
 Roff = Roff - _args[3]  ;
         }
 }
     net_send ( _tqitem, _args, _pnt, t +  Cdur , _args[2] ) ;
     }
   if ( _lflag  == _args[2] ) {
     _args[3] = weight * Rinf * _args[6] + ( _args[3] - weight * Rinf * _args[6] ) * exp ( - ( t - _args[7] ) / Rtau ) ;
     _args[7] = t ;
     synon = synon - weight * _args[6] ;
         if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = Ron;
    double __primary = (Ron - _args[3] ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( ( ( - 1.0 ) ) ) / Rtau ) ) )*( - ( ( ( ( synon )*( Rinf ) ) ) / Rtau ) / ( ( ( ( - 1.0 ) ) ) / Rtau ) - __primary );
    Ron += __primary;
  } else {
 Ron = Ron - _args[3]  ;
       }
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = Roff;
    double __primary = (Roff + _args[3] ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - Beta )*( 1.0 ) ) ) )*( - ( 0.0 ) / ( ( - Beta )*( 1.0 ) ) - __primary );
    Roff += __primary;
  } else {
 Roff = Roff + _args[3]  ;
       }
 _args[1] = 0.0 ;
     }
   _args[4] = _args[4] + x * _args[6] ;
   } }
 
static void _net_init(Point_process* _pnt, double* _args, double _lflag) {
       _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
 _args[4] = 0.0 ;
   _args[5] = 0.0 ;
   _args[6] = u0 ;
   _args[7] = t ;
   }
 
double lthresh (  double _lconc , double _lKD , double _ln ) {
   double _llthresh;
 _llthresh = pow( _lconc , _ln ) / ( pow( _lKD , _ln ) + pow( _lconc , _ln ) ) ;
   
return _llthresh;
 }
 
static double _hoc_lthresh(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  lthresh (  *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double hthresh (  double _lconc , double _lKD , double _ln ) {
   double _lhthresh;
 _lhthresh = pow( _lKD , _ln ) / ( pow( _lKD , _ln ) + pow( _lconc , _ln ) ) ;
   
return _lhthresh;
 }
 
static double _hoc_hthresh(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  hthresh (  *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double sigmoidal (  double _lx , double _lx_offset , double _ls ) {
   double _lsigmoidal;
 _lsigmoidal = 1.0 / ( 1.0 + exp ( - _ls * ( _lx - _lx_offset ) ) ) ;
   
return _lsigmoidal;
 }
 
static double _hoc_sigmoidal(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  sigmoidal (  *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double quadratic (  double _lx , double _lx1 , double _lx2 ) {
   double _lquadratic;
 _lquadratic = - 1.0 * ( _lx - _lx1 ) * ( _lx - _lx2 ) ;
   
return _lquadratic;
 }
 
static double _hoc_quadratic(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  quadratic (  *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double max (  double _lcurrent , double _lmaximum ) {
   double _lmax;
 if ( _lcurrent > _lmaximum ) {
     _lmax = _lcurrent ;
     }
   else {
     _lmax = _lmaximum ;
     }
   
return _lmax;
 }
 
static double _hoc_max(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  max (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double min (  double _lcurrent , double _lminimum ) {
   double _lmin;
 if ( _lcurrent < _lminimum ) {
     _lmin = _lcurrent ;
     }
   else {
     _lmin = _lminimum ;
     }
   
return _lmin;
 }
 
static double _hoc_min(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  min (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
static int  reset_max (  ) {
   ca_nmdai_max = 0.0 ;
   cali_max = 0.0 ;
   cati_max = 0.0 ;
   active_syn_flag = 0.0 ;
    return 0; }
 
static double _hoc_reset_max(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r = 1.;
 reset_max (  );
 return(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ca_nmdai = _ion_ca_nmdai;
  cali = _ion_cali;
  cati = _ion_cati;
     _ode_spec1 ();
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ca_nmdai = _ion_ca_nmdai;
  cali = _ion_cali;
  cati = _ion_cati;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 2, 1);
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 3, 3);
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 4, 4);
   nrn_update_ion_pointer(_cal_sym, _ppvar, 5, 1);
   nrn_update_ion_pointer(_cat_sym, _ppvar, 6, 1);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  Roff = Roff0;
  Ron = Ron0;
 {
   Rinf = Cmax * Alpha / ( Cmax * Alpha + Beta ) ;
   Rtau = 1.0 / ( Cmax * Alpha + Beta ) ;
   synon = 0.0 ;
   weight = w0 ;
   last_dopamine = 0.0 ;
   lthresh_LTP = KD1 ;
   hthresh_LTP = KD2 ;
   lthresh_LTD = KD_LTD ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  ca_nmdai = _ion_ca_nmdai;
  cali = _ion_cali;
  cati = _ion_cati;
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   double _lnorm ;
 B = mgblock ( _threadargscomma_ v ) ;
   g = ( Ron + Roff ) * gmax * B ;
   iNMDA = g * ( v - Erev ) ;
   ica_nmda = nmda_ca_fraction * iNMDA ;
   iNMDA = ( 1.0 - nmda_ca_fraction ) * iNMDA ;
   if ( stimulus_flag  == 1.0 ) {
     ca_nmdai_max = max ( _threadargscomma_ ca_nmdai , ca_nmdai_max ) ;
     cali_max = max ( _threadargscomma_ cali , cali_max ) ;
     cati_max = max ( _threadargscomma_ cati , cati_max ) ;
     last_dopamine = dopamine ;
     }
   else {
     _lnorm = sigmoidal ( _threadargscomma_ 0.5 * ( lthresh_LTP + hthresh_LTP ) , lthresh_LTP , n1 ) * ( 1.0 - sigmoidal ( _threadargscomma_ 0.5 * ( lthresh_LTP + hthresh_LTP ) , hthresh_LTP , n2 ) ) ;
     kernel = sigmoidal ( _threadargscomma_ ca_nmdai_max , lthresh_LTP , n1 ) * ( 1.0 - sigmoidal ( _threadargscomma_ ca_nmdai_max , hthresh_LTP , n2 ) ) * 1.0 / _lnorm ;
     if ( kernel < 0.01 ) {
       kernel = 0.0 ;
       }
     if ( last_dopamine  == 1.0  && active_syn_flag  == 1.0 ) {
       weight = weight + learning_rate_w_LTP * kernel ;
       lthresh_LTP = lthresh_LTP - learning_rate_thresh_LTP * kernel ;
       hthresh_LTP = lthresh_LTP + ( KD2 - KD1 ) ;
       }
     else if ( last_dopamine  == - 1.0  && active_syn_flag  == 1.0 ) {
       weight = weight - learning_rate_w_LTD * sigmoidal ( _threadargscomma_ cali_max , KD_LTD , n_LTD ) * weight ;
       lthresh_LTP = lthresh_LTP + learning_rate_thresh_LTD * kernel * sigmoidal ( _threadargscomma_ cali_max , KD_LTD , n_LTD ) ;
       hthresh_LTP = lthresh_LTP + ( KD2 - KD1 ) ;
       }
     last_dopamine = dopamine ;
     reset_max ( _threadargs_ ) ;
     }
   }
 _current += iNMDA;
 _current += ica_nmda;

} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  ca_nmdai = _ion_ca_nmdai;
  cali = _ion_cali;
  cati = _ion_cati;
 _g = _nrn_current(_v + .001);
 	{ double _dica_nmda;
  _dica_nmda = ica_nmda;
 _rhs = _nrn_current(_v);
  _ion_dica_nmdadv += (_dica_nmda - ica_nmda)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica_nmda += ica_nmda * 1.e2/ (_nd_area);
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 
}}

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  ca_nmdai = _ion_ca_nmdai;
  cali = _ion_cali;
  cati = _ion_cati;
 { error =  release();
 if(error){fprintf(stderr,"at line 164 in file adaptive_shom_NMDA_stp.mod:\n	SOLVE release METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 } }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = Ron_columnindex;  _dlist1[0] = DRon_columnindex;
 _slist1[1] = Roff_columnindex;  _dlist1[1] = DRoff_columnindex;
   _t_mgblock = makevector(1001*sizeof(double));
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "/home/shadeform/Spillover_2/mod/adaptive_shom_NMDA_stp.mod";
static const char* nmodl_file_text = 
  "\n"
  "\n"
  "TITLE simple NMDA receptors\n"
  "\n"
  "COMMENT\n"
  "-----------------------------------------------------------------------------\n"
  "\n"
  "Essentially the same as /examples/nrniv/netcon/ampa.mod in the NEURON\n"
  "distribution - i.e. Alain Destexhe's simple AMPA model - but with\n"
  "different binding and unbinding rates and with a magnesium block.\n"
  "Modified by Andrew Davison, The Babraham Institute, May 2000\n"
  "\n"
  "\n"
  "	Simple model for glutamate AMPA receptors\n"
  "	=========================================\n"
  "\n"
  "  - FIRST-ORDER KINETICS, FIT TO WHOLE-CELL RECORDINGS\n"
  "\n"
  "    Whole-cell recorded postsynaptic currents mediated by AMPA/Kainate\n"
  "    receptors (Xiang et al., J. Neurophysiol. 71: 2552-2556, 1994) were used\n"
  "    to estimate the parameters of the present model; the fit was performed\n"
  "    using a simplex algorithm (see Destexhe et al., J. Computational Neurosci.\n"
  "    1: 195-230, 1994).\n"
  "\n"
  "  - SHORT PULSES OF TRANSMITTER (0.3 ms, 0.5 mM)\n"
  "\n"
  "    The simplified model was obtained from a detailed synaptic model that\n"
  "    included the release of transmitter in adjacent terminals, its lateral\n"
  "    diffusion and uptake, and its binding on postsynaptic receptors (Destexhe\n"
  "    and Sejnowski, 1995).  Short pulses of transmitter with first-order\n"
  "    kinetics were found to be the best fast alternative to represent the more\n"
  "    detailed models.\n"
  "\n"
  "  - ANALYTIC EXPRESSION\n"
  "\n"
  "    The first-order model can be solved analytically, leading to a very fast\n"
  "    mechanism for simulating synapses, since no differential equation must be\n"
  "    solved (see references below).\n"
  "\n"
  "\n"
  "\n"
  "References\n"
  "\n"
  "   Destexhe, A., Mainen, Z.F. and Sejnowski, T.J.  An efficient method for\n"
  "   computing synaptic conductances based on a kinetic model of receptor binding\n"
  "   Neural Computation 6: 10-14, 1994.\n"
  "\n"
  "   Destexhe, A., Mainen, Z.F. and Sejnowski, T.J. Synthesis of models for\n"
  "   excitable membranes, synaptic transmission and neuromodulation using a\n"
  "   common kinetic formalism, Journal of Computational Neuroscience 1:\n"
  "   195-230, 1994.\n"
  "\n"
  "Orignal file by:\n"
  "Kiki Sidiropoulou\n"
  "Adjusted Cdur = 1 and Beta= 0.01 for better nmda spikes\n"
  "PROCEDURE rate: FROM -140 TO 80 WITH 1000\n"
  "\n"
  "Modified by Penny under the instruction of M.L.Hines on Oct 03, 2017\n"
  "	Change gmax\n"
  "\n"
  "-----------------------------------------------------------------------------\n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS adaptive_shom_NMDA_stp\n"
  "	RANGE g, Alpha, Beta, Erev, gmax, Cdur, iNMDA\n"
  "	NONSPECIFIC_CURRENT  iNMDA\n"
  "	RANGE mg, Cmax, eta, alpha, nmda_ca_fraction, u0, U\n"
  "        POINTER dopamine, stimulus_flag\n"
  "	RANGE learning_rate_w_LTP, learning_rate_w_LTD, learning_rate_thresh_LTP, learning_rate_thresh_LTD, learning_rate_thresh_KD_LTD\n"
  "	RANGE ca_nmdai_max, cali_max, cati_max, active_syn_flag, Cdur_init, Cdur_factor, w0, kernel\n"
  "        RANGE last_dopamine, weight, KD1, KD2, n1, n2, KD_LTD, n_LTD, lthresh_LTP, hthresh_LTP, lthresh_LTD\n"
  "	USEION ca_nmda READ ca_nmdai WRITE ica_nmda VALENCE 2	\n"
  "	USEION cal READ cali VALENCE 2\n"
  "	USEION cat READ cati VALENCE 2\n"
  "\n"
  "}\n"
  "UNITS {\n"
  "	(nA) = (nanoamp)\n"
  "	(mV) = (millivolt)\n"
  "	(uS) = (microsiemens)\n"
  "	(mM) = (milli/liter)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	Cmax	= 1	 (mM)           : max transmitter concentration\n"
  "	Alpha	= 4 (/ms /mM)	: forward (binding) rate (4)\n"
  "	Beta 	= 0.01   (/ms)   : backward (unbinding) rate\n"
  "	Erev	= 0	 (mV)		: reversal potential\n"
  "        mg   = 1      (mM)           : external magnesium concentration\n"
  "        eta = 0.28 (/mV)\n"
  "        alpha = 0.072 (/mV)\n"
  "	gmax = 1   (uS)\n"
  "\n"
  "	learning_rate_w_LTP = 0.01\n"
  "    	learning_rate_w_LTD = 0.01\n"
  "    	learning_rate_thresh_LTP = 0.01\n"
  "    	learning_rate_thresh_LTD = 0.01\n"
  "    	learning_rate_thresh_KD_LTD = 0.01\n"
  "    	wmax = 0.006 (uS)\n"
  "    	wmin = 0.001 (uS)\n"
  "        w0 = 0.00188 (uS)\n"
  "	ca_nmdai_max = 0\n"
  "	cali_max = 0\n"
  "	cati_max = 0\n"
  "	active_syn_flag = 0\n"
  "        nmda_ca_fraction = 0.15\n"
  "        Cdur = 1.1\n"
  "	n1 = 5\n"
  "	n2 = 5\n"
  "	KD1 = 1\n"
  "	KD2 = 1\n"
  "	KD_LTD = 1\n"
  "	n_LTD = 1\n"
  "\n"
  "        tau = 3 (ms)\n"
  "        tauR = 100 (ms)  : tauR > tau\n"
  "        tauF = 0 (ms)  : tauF >= 0 (org: 800 ms)\n"
  "        U = 0.3 (1) <0, 1>\n"
  "        u0 = 0 (1) <0, 1>\n"
  "}\n"
  "\n"
  "\n"
  "ASSIGNED {\n"
  "	v		(mV)		: postsynaptic voltage\n"
  "	iNMDA 		(nA)		: current = g*(v - e)\n"
  "	g 		(uS)		: conductance\n"
  "	Rinf				: steady state channels open\n"
  "	Rtau		(ms)		: time constant of channel binding\n"
  "	synon\n"
  "        B                       : magnesium block\n"
  "	ica_nmda        (nA)\n"
  "	dopamine\n"
  "        stimulus_flag\n"
  "        cati            (mM)\n"
  "        cali            (mM)\n"
  "	ca_nmdai        (mM)\n"
  "        last_dopamine\n"
  "        weight\n"
  "        lthresh_LTP\n"
  "        hthresh_LTP\n"
  "        lthresh_LTD\n"
  "        kernel\n"
  "        x\n"
  "}\n"
  "\n"
  "STATE {Ron Roff}\n"
  "\n"
  "INITIAL {\n"
  "	Rinf = Cmax*Alpha / (Cmax*Alpha + Beta)\n"
  "	Rtau = 1 / (Cmax*Alpha + Beta)\n"
  "	synon = 0\n"
  "	weight = w0\n"
  "        last_dopamine = 0\n"
  "        lthresh_LTP = KD1\n"
  "        hthresh_LTP = KD2\n"
  "        lthresh_LTD = KD_LTD\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "        LOCAL norm\n"
  "	SOLVE release METHOD cnexp\n"
  "        B = mgblock(v)\n"
  "	g = (Ron + Roff)* gmax * B\n"
  "	iNMDA = g*(v - Erev)\n"
  "        ica_nmda = nmda_ca_fraction*iNMDA\n"
  "        iNMDA = (1 - nmda_ca_fraction)*iNMDA\n"
  "    \n"
  "        if (stimulus_flag == 1) {\n"
  "        	ca_nmdai_max = max(ca_nmdai, ca_nmdai_max)\n"
  "        	cali_max = max(cali, cali_max)\n"
  "        	cati_max = max(cati, cati_max)\n"
  "		last_dopamine = dopamine\n"
  "        } else {\n"
  "        	norm = sigmoidal(0.5*(lthresh_LTP+hthresh_LTP), lthresh_LTP, n1)* (1 - sigmoidal(0.5*(lthresh_LTP+hthresh_LTP), hthresh_LTP, n2))\n"
  "                kernel = sigmoidal(ca_nmdai_max, lthresh_LTP, n1)* (1 - sigmoidal(ca_nmdai_max, hthresh_LTP, n2))*1.0/norm\n"
  "		 if (kernel < 0.01) { kernel = 0 }\n"
  "		  \n"
  "		 :norm = quadratic(0.5*(lthresh_LTP+hthresh_LTP), lthresh_LTP, hthresh_LTP)\n"
  "                :kernel = quadratic(ca_nmdai_max, lthresh_LTP, hthresh_LTP)/norm\n"
  "		 :if  (kernel < 0) {kernel = 0}\n"
  "	  \n"
  "	  if (last_dopamine == 1 && active_syn_flag == 1) {\n"
  "                 weight = weight + learning_rate_w_LTP * kernel \n"
  "		  lthresh_LTP = lthresh_LTP - learning_rate_thresh_LTP * kernel\n"
  "		  hthresh_LTP = lthresh_LTP + (KD2 - KD1)\n"
  "		  :hthresh_LTP = max(hthresh_LTP - learning_rate_thresh_LTP * kernel, lthresh_LTP)		  \n"
  "          } else if (last_dopamine == -1 && active_syn_flag == 1) {\n"
  "		  \n"
  "		  weight = weight - learning_rate_w_LTD * sigmoidal(cali_max, KD_LTD, n_LTD) * weight\n"
  "		  lthresh_LTP = lthresh_LTP + learning_rate_thresh_LTD * kernel * sigmoidal(cali_max, KD_LTD,n_LTD)\n"
  "		  hthresh_LTP = lthresh_LTP + (KD2 - KD1)\n"
  "		  \n"
  "		  :lthresh_LTD = max(KD_LTD, lthresh_LTD + learning_rate_thresh_KD_LTD * (cali_max - lthresh_LTD))\n"
  "		  :lthresh_LTP = max(lthresh_LTP - learning_rate_thresh_LTD * sigmoidal(cali_max, KD_LTD, n_LTD) * lthresh_LTP, lthresh_LTP_min)\n"
  "		  :hthresh_LTP = max(hthresh_LTP + learning_rate_thresh_LTD * kernel * sigmoidal(cali_max, KD_LTD,n_LTD), lthresh_LTP)\n"
  "          }\n"
  "          last_dopamine = dopamine		\n"
  "          reset_max()\n"
  "        }\n"
  "\n"
  "}\n"
  "\n"
  "DERIVATIVE release {\n"
  "	Ron' = (synon*Rinf - Ron)/Rtau\n"
  "	Roff' = -Beta*Roff\n"
  "}\n"
  "\n"
  "FUNCTION mgblock(v(mV)) {\n"
  "        TABLE\n"
  "        DEPEND mg\n"
  "        FROM -140 TO 80 WITH 1000\n"
  "\n"
  "        : from Jahr & Stevens\n"
  "\n"
  "\n"
  "	 mgblock = 1 / (1 + mg * eta * exp(-alpha * v) )  :was 0.062, changed to 0.072 to get a better voltage-dependence of NMDA currents, july 2008, kiki\n"
  "\n"
  "}\n"
  "\n"
  ": following supports both saturation from single input and\n"
  ": summation from multiple inputs\n"
  ": if spike occurs during CDur then new off time is t + CDur\n"
  ": ie. transmitter concatenates but does not summate\n"
  ": Note: automatic initialization of all reference args to 0 except first\n"
  "\n"
  "\n"
  "NET_RECEIVE(dummy, on, nspike, r0, y, z, u, t0 (ms)) {\n"
  "	INITIAL {\n"
  "		y = 0\n"
  "		z = 0\n"
  "		u = u0\n"
  "		t0 = t\n"
  "	}\n"
  "	z = z*exp(-(t-t0)/tauR)\n"
  "	z = z + (y*(exp(-(t-t0)/tau) - exp(-(t-t0)/tauR)) / (tau/tauR - 1) )\n"
  "	y = y*exp(-(t-t0)/tau)\n"
  "	x = 1-y-z\n"
  "	if (tauF > 0) {\n"
  "		u = u*exp(-(t-t0)/tauF)\n"
  "		u = u + U*(1-u)\n"
  "	} else {\n"
  "		u = U\n"
  "	}\n"
  "	\n"
  "	: flag is an implicit argument of NET_RECEIVE and  normally 0\n"
  "        if (flag == 0) { : a spike, so turn on if not already in a Cdur pulse\n"
  "		active_syn_flag = 1\n"
  "		nspike = nspike + 1\n"
  "		if (!on) {\n"
  "			r0 = r0*exp(-Beta*(t - t0))\n"
  "			t0 = t\n"
  "			on = 1\n"
  "			synon = synon + weight*u\n"
  "			state_discontinuity(Ron, Ron + r0)\n"
  "			state_discontinuity(Roff, Roff - r0)\n"
  "		}\n"
  ":		 come again in Cdur with flag = current value of nspike\n"
  "		net_send(Cdur, nspike)\n"
  "       }\n"
  "	if (flag == nspike) { : if this associated with last spike then turn off\n"
  "		r0 = weight*Rinf*u + (r0 - weight*Rinf*u)*exp(-(t - t0)/Rtau)\n"
  "		t0 = t\n"
  "		synon = synon - weight*u\n"
  "		state_discontinuity(Ron, Ron - r0)\n"
  "		state_discontinuity(Roff, Roff + r0)\n"
  "		on = 0\n"
  "	}\n"
  "	y = y + x*u\n"
  "}\n"
  "\n"
  "FUNCTION lthresh(conc, KD, n) {\n"
  "    lthresh = conc^n/(KD^n + conc^n) \n"
  "}\n"
  "\n"
  "FUNCTION hthresh(conc, KD, n) {\n"
  "    hthresh = KD^n/(KD^n + conc^n) \n"
  "}\n"
  "\n"
  "FUNCTION sigmoidal(x, x_offset, s) {\n"
  "    sigmoidal = 1/(1+exp(-s *(x - x_offset)))\n"
  "}\n"
  "\n"
  "FUNCTION quadratic(x, x1, x2) {\n"
  "    quadratic = -1*(x-x1)*(x-x2)\n"
  "}\n"
  "\n"
  "FUNCTION max(current, maximum) {\n"
  "   if (current>maximum) { \n"
  "      max = current\n"
  "   } else {\n"
  "      max = maximum\n"
  "   }\n"
  "}\n"
  "\n"
  "FUNCTION min(current, minimum) {\n"
  "   if (current<minimum) { \n"
  "      min = current\n"
  "   } else {\n"
  "      min = minimum\n"
  "   }\n"
  "}\n"
  "\n"
  "\n"
  "PROCEDURE reset_max() {\n"
  "	ca_nmdai_max = 0\n"
  "        cali_max = 0\n"
  "        cati_max = 0\n"
  "        active_syn_flag = 0\n"
  "}\n"
  ;
#endif
