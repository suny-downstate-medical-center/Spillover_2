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
 
#define nrn_init _nrn_init__adaptive_shom_NMDA
#define _nrn_initial _nrn_initial__adaptive_shom_NMDA
#define nrn_cur _nrn_cur__adaptive_shom_NMDA
#define _nrn_current _nrn_current__adaptive_shom_NMDA
#define nrn_jacob _nrn_jacob__adaptive_shom_NMDA
#define nrn_state _nrn_state__adaptive_shom_NMDA
#define _net_receive _net_receive__adaptive_shom_NMDA 
#define release release__adaptive_shom_NMDA 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define Cmax _p[0]
#define Cmax_columnindex 0
#define Cdur _p[1]
#define Cdur_columnindex 1
#define Alpha _p[2]
#define Alpha_columnindex 2
#define Beta _p[3]
#define Beta_columnindex 3
#define Erev _p[4]
#define Erev_columnindex 4
#define mg _p[5]
#define mg_columnindex 5
#define eta _p[6]
#define eta_columnindex 6
#define alpha _p[7]
#define alpha_columnindex 7
#define gmax _p[8]
#define gmax_columnindex 8
#define w0 _p[9]
#define w0_columnindex 9
#define conc0 _p[10]
#define conc0_columnindex 10
#define flagx _p[11]
#define flagx_columnindex 11
#define mltype _p[12]
#define mltype_columnindex 12
#define mltypeMin _p[13]
#define mltypeMin_columnindex 13
#define learning_rate_w_LTD _p[14]
#define learning_rate_w_LTD_columnindex 14
#define learning_rate_w_LTP _p[15]
#define learning_rate_w_LTP_columnindex 15
#define learning_rate_thresh_LTD _p[16]
#define learning_rate_thresh_LTD_columnindex 16
#define learning_rate_thresh_LTP _p[17]
#define learning_rate_thresh_LTP_columnindex 17
#define learning_rate_thresh_KD_LTD _p[18]
#define learning_rate_thresh_KD_LTD_columnindex 18
#define threshf _p[19]
#define threshf_columnindex 19
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
#define nmda_ca_fraction _p[26]
#define nmda_ca_fraction_columnindex 26
#define kernel_LTD _p[27]
#define kernel_LTD_columnindex 27
#define cai_max _p[28]
#define cai_max_columnindex 28
#define ca_nmdai_max _p[29]
#define ca_nmdai_max_columnindex 29
#define cali_max _p[30]
#define cali_max_columnindex 30
#define cati_max _p[31]
#define cati_max_columnindex 31
#define iNMDA _p[32]
#define iNMDA_columnindex 32
#define g _p[33]
#define g_columnindex 33
#define synon _p[34]
#define synon_columnindex 34
#define kernel _p[35]
#define kernel_columnindex 35
#define lthresh_LTP _p[36]
#define lthresh_LTP_columnindex 36
#define hthresh_LTP _p[37]
#define hthresh_LTP_columnindex 37
#define lthresh_LTD _p[38]
#define lthresh_LTD_columnindex 38
#define Ron _p[39]
#define Ron_columnindex 39
#define Roff _p[40]
#define Roff_columnindex 40
#define weight _p[41]
#define weight_columnindex 41
#define thresh _p[42]
#define thresh_columnindex 42
#define Rinf _p[43]
#define Rinf_columnindex 43
#define Rtau _p[44]
#define Rtau_columnindex 44
#define B _p[45]
#define B_columnindex 45
#define ica _p[46]
#define ica_columnindex 46
#define cali _p[47]
#define cali_columnindex 47
#define ca_nmdai _p[48]
#define ca_nmdai_columnindex 48
#define cai _p[49]
#define cai_columnindex 49
#define cati _p[50]
#define cati_columnindex 50
#define DRon _p[51]
#define DRon_columnindex 51
#define DRoff _p[52]
#define DRoff_columnindex 52
#define Dweight _p[53]
#define Dweight_columnindex 53
#define Dthresh _p[54]
#define Dthresh_columnindex 54
#define _g _p[55]
#define _g_columnindex 55
#define _tsav _p[56]
#define _tsav_columnindex 56
#define _nd_area  *_ppvar[0]._pval
#define _ion_ca_nmdai	*_ppvar[2]._pval
#define _ion_cali	*_ppvar[3]._pval
#define _ion_cati	*_ppvar[4]._pval
#define _ion_cai	*_ppvar[5]._pval
#define _ion_ica	*_ppvar[6]._pval
#define _ion_dicadv	*_ppvar[7]._pval
#define dopamine	*_ppvar[8]._pval
#define _p_dopamine	_ppvar[8]._pval
#define stimulus_flag	*_ppvar[9]._pval
#define _p_stimulus_flag	_ppvar[9]._pval
 
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
 static int hoc_nrnpointerindex =  8;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_Norm(void*);
 static double _hoc_funcCalMin1(void*);
 static double _hoc_funcCalteg(void*);
 static double _hoc_funcCalt(void*);
 static double _hoc_funcCal(void*);
 static double _hoc_funcCalMin(void*);
 static double _hoc_mgblock(void*);
 static double _hoc_sTresh(void*);
 static double _hoc_supra(void*);
 static double _hoc_trap(void*);
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
 "Norm", _hoc_Norm,
 "funcCalMin1", _hoc_funcCalMin1,
 "funcCalteg", _hoc_funcCalteg,
 "funcCalt", _hoc_funcCalt,
 "funcCal", _hoc_funcCal,
 "funcCalMin", _hoc_funcCalMin,
 "mgblock", _hoc_mgblock,
 "sTresh", _hoc_sTresh,
 "supra", _hoc_supra,
 "trap", _hoc_trap,
 0, 0
};
#define Norm Norm_adaptive_shom_NMDA
#define _f_mgblock _f_mgblock_adaptive_shom_NMDA
#define funcCalMin1 funcCalMin1_adaptive_shom_NMDA
#define funcCalteg funcCalteg_adaptive_shom_NMDA
#define funcCalt funcCalt_adaptive_shom_NMDA
#define funcCal funcCal_adaptive_shom_NMDA
#define funcCalMin funcCalMin_adaptive_shom_NMDA
#define mgblock mgblock_adaptive_shom_NMDA
#define sTresh sTresh_adaptive_shom_NMDA
#define supra supra_adaptive_shom_NMDA
#define trap trap_adaptive_shom_NMDA
 extern double Norm( double );
 extern double _f_mgblock( double );
 extern double funcCalMin1( double );
 extern double funcCalteg( double , double );
 extern double funcCalt( double , double );
 extern double funcCal( double , double );
 extern double funcCalMin( double );
 extern double mgblock( double );
 extern double sTresh( double , double , double );
 extern double supra( double , double , double , double , double , double , double );
 extern double trap( double );
 /* declare global and static user variables */
#define conc conc_adaptive_shom_NMDA
 double conc = 0;
#define f f_adaptive_shom_NMDA
 double f = 0;
#define threshltp threshltp_adaptive_shom_NMDA
 double threshltp = 0.02;
#define usetable usetable_adaptive_shom_NMDA
 double usetable = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "usetable_adaptive_shom_NMDA", 0, 1,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "threshltp_adaptive_shom_NMDA", "mM",
 "f_adaptive_shom_NMDA", "ms",
 "conc_adaptive_shom_NMDA", "mM",
 "Cmax", "mM",
 "Cdur", "ms",
 "Alpha", "/ms",
 "Beta", "/ms",
 "Erev", "mV",
 "mg", "mM",
 "alpha", "/mV",
 "gmax", "uS",
 "conc0", "mM",
 "flagx", "1",
 "mltype", "mM",
 "mltypeMin", "mM",
 "learning_rate_w_LTD", "1",
 "learning_rate_w_LTP", "1",
 "learning_rate_thresh_LTD", "1",
 "learning_rate_thresh_LTP", "1",
 "threshf", "mM",
 "iNMDA", "nA",
 "g", "uS",
 0,0
};
 static double Roff0 = 0;
 static double Ron0 = 0;
 static double delta_t = 0.01;
 static double thresh0 = 0;
 static double v = 0;
 static double weight0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "threshltp_adaptive_shom_NMDA", &threshltp_adaptive_shom_NMDA,
 "f_adaptive_shom_NMDA", &f_adaptive_shom_NMDA,
 "conc_adaptive_shom_NMDA", &conc_adaptive_shom_NMDA,
 "usetable_adaptive_shom_NMDA", &usetable_adaptive_shom_NMDA,
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
 
#define _cvode_ieq _ppvar[11]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"adaptive_shom_NMDA",
 "Cmax",
 "Cdur",
 "Alpha",
 "Beta",
 "Erev",
 "mg",
 "eta",
 "alpha",
 "gmax",
 "w0",
 "conc0",
 "flagx",
 "mltype",
 "mltypeMin",
 "learning_rate_w_LTD",
 "learning_rate_w_LTP",
 "learning_rate_thresh_LTD",
 "learning_rate_thresh_LTP",
 "learning_rate_thresh_KD_LTD",
 "threshf",
 "n1",
 "n2",
 "KD1",
 "KD2",
 "KD_LTD",
 "n_LTD",
 "nmda_ca_fraction",
 "kernel_LTD",
 "cai_max",
 "ca_nmdai_max",
 "cali_max",
 "cati_max",
 0,
 "iNMDA",
 "g",
 "synon",
 "kernel",
 "lthresh_LTP",
 "hthresh_LTP",
 "lthresh_LTD",
 0,
 "Ron",
 "Roff",
 "weight",
 "thresh",
 0,
 "dopamine",
 "stimulus_flag",
 0};
 static Symbol* _ca_nmda_sym;
 static Symbol* _cal_sym;
 static Symbol* _cat_sym;
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 57, _prop);
 	/*initialize range parameters*/
 	Cmax = 1;
 	Cdur = 1;
 	Alpha = 4;
 	Beta = 0.01;
 	Erev = 15;
 	mg = 0;
 	eta = 0;
 	alpha = 0;
 	gmax = 0;
 	w0 = 0;
 	conc0 = 0;
 	flagx = 0;
 	mltype = 0;
 	mltypeMin = 0.00015;
 	learning_rate_w_LTD = 0;
 	learning_rate_w_LTP = 0;
 	learning_rate_thresh_LTD = 0;
 	learning_rate_thresh_LTP = 0;
 	learning_rate_thresh_KD_LTD = 0;
 	threshf = 0;
 	n1 = 5;
 	n2 = 5;
 	KD1 = 1;
 	KD2 = 1;
 	KD_LTD = 1;
 	n_LTD = 1;
 	nmda_ca_fraction = 0.175;
 	kernel_LTD = 0;
 	cai_max = 0;
 	ca_nmdai_max = 0;
 	cali_max = 0;
 	cati_max = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 57;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 12, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_nmda_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2]._pval = &prop_ion->param[1]; /* ca_nmdai */
 prop_ion = need_memb(_cal_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[3]._pval = &prop_ion->param[1]; /* cali */
 prop_ion = need_memb(_cat_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[4]._pval = &prop_ion->param[1]; /* cati */
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[5]._pval = &prop_ion->param[1]; /* cai */
 	_ppvar[6]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[7]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 
#define _tqitem &(_ppvar[10]._pvoid)
 static void _net_receive(Point_process*, double*, double);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _adaptive_shom_NMDA_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca_nmda", 2.0);
 	ion_reg("cal", 2.0);
 	ion_reg("cat", 2.0);
 	ion_reg("ca", 2.0);
 	_ca_nmda_sym = hoc_lookup("ca_nmda_ion");
 	_cal_sym = hoc_lookup("cal_ion");
 	_cat_sym = hoc_lookup("cat_ion");
 	_ca_sym = hoc_lookup("ca_ion");
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
  hoc_register_prop_size(_mechtype, 57, 12);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "cat_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 7, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 8, "pointer");
  hoc_register_dparam_semantics(_mechtype, 9, "pointer");
  hoc_register_dparam_semantics(_mechtype, 10, "netsend");
  hoc_register_dparam_semantics(_mechtype, 11, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 5;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 adaptive_shom_NMDA /mnt/data/Spillover_2/mod/adaptive_shom_NMDA.mod\n");
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
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = 0;}
 {
   if ( _lflag  == 0.0 ) {
     _args[2] = _args[2] + 1.0 ;
     if (  ! _args[1] ) {
       _args[3] = _args[3] * exp ( - Beta * ( t - _args[4] ) ) ;
       _args[4] = t ;
       _args[1] = 1.0 ;
       synon = synon + weight ;
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
     _args[3] = weight * Rinf + ( _args[3] - weight * Rinf ) * exp ( - ( t - _args[4] ) / Rtau ) ;
     _args[4] = t ;
     synon = synon - weight ;
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
   } }
 
double supra (  double _ldopam , double _lwe , double _lconc , double _lcalic , double _lcai , double _lg_p , double _ltresh ) {
   double _lsupra;
  if ( _lg_p < 1e-7 ) {
     conc0 = 0.0 ;
     mltype = 0.0 ;
     }
   else {
     if ( _lconc > conc0 ) {
       conc0 = _lconc ;
       }
     if ( _lcalic > mltype ) {
       mltype = _lcalic ;
       }
     }
   _lsupra = ( ( learning_rate_w_LTD * _lwe * mltype * funcCalMin ( _threadargscomma_ mltype ) * ( _ldopam - 1.0 ) ) + ( learning_rate_w_LTP * funcCal ( _threadargscomma_ conc0 , _ltresh ) * ( _ldopam + 1.0 ) ) ) * trap ( _threadargscomma_ _lg_p ) * _ldopam * _ldopam ;
    
return _lsupra;
 }
 
static double _hoc_supra(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  supra (  *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) , *getarg(6) , *getarg(7) );
 return(_r);
}
 
double sTresh (  double _ldopam , double _ltresh , double _lconc0 ) {
   double _lsTresh;
  _lsTresh = ( ( - learning_rate_thresh_LTD * funcCalt ( _threadargscomma_ _lconc0 , _ltresh ) * ( _ldopam - 1.0 ) ) - learning_rate_thresh_LTP * ( funcCalt ( _threadargscomma_ _lconc0 , _ltresh ) * ( _ldopam + 1.0 ) ) ) * trap ( _threadargscomma_ g ) * _ldopam * _ldopam ;
    
return _lsTresh;
 }
 
static double _hoc_sTresh(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  sTresh (  *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double trap (  double _lg_p ) {
   double _ltrap;
  if ( _lg_p < 1e-7 ) {
     _ltrap = 0.0 ;
     }
   else {
     _ltrap = 1.0 ;
     }
    
return _ltrap;
 }
 
static double _hoc_trap(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  trap (  *getarg(1) );
 return(_r);
}
 
double funcCal (  double _lcalnm , double _ltresh ) {
   double _lfuncCal;
  _lfuncCal = ( 1.0 - ( 1.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 1.0 ) ) ) ) * ( 1.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 1.0 ) ) ) ;
    
return _lfuncCal;
 }
 
static double _hoc_funcCal(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  funcCal (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double funcCalt (  double _lcalnm , double _ltresh ) {
   double _lfuncCalt;
  _lfuncCalt = ( 1.0 - ( 1.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 3.0 ) ) ) ) * ( 1.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 3.0 ) ) ) ;
    
return _lfuncCalt;
 }
 
static double _hoc_funcCalt(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  funcCalt (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double funcCalteg (  double _lcalnm , double _ltresh ) {
   double _lfuncCalteg;
  _lfuncCalteg = ( - 1.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 2.5 ) ) ) ;
    
return _lfuncCalteg;
 }
 
static double _hoc_funcCalteg(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  funcCalteg (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double funcCalMin (  double _lcalnm ) {
   double _lfuncCalMin;
  _lfuncCalMin = 1.0 / ( 1.0 + exp ( ( - Norm ( _threadargscomma_ _lcalnm ) + 7.0 ) / 1.0 ) ) ;
    
return _lfuncCalMin;
 }
 
static double _hoc_funcCalMin(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  funcCalMin (  *getarg(1) );
 return(_r);
}
 
double Norm (  double _lcalnm ) {
   double _lNorm;
  _lNorm = 100000.0 * _lcalnm ;
    
return _lNorm;
 }
 
static double _hoc_Norm(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  Norm (  *getarg(1) );
 return(_r);
}
 
double funcCalMin1 (  double _lcalnm ) {
   double _lfuncCalMin1;
  _lfuncCalMin1 = 1.0 / ( 1.0 + exp ( ( - 100000.0 * _lcalnm + ( 100000.0 * mltypeMin ) ) / 1.0 ) ) ;
    
return _lfuncCalMin1;
 }
 
static double _hoc_funcCalMin1(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  funcCalMin1 (  *getarg(1) );
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
  cai = _ion_cai;
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
  cai = _ion_cai;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 2, 1);
   nrn_update_ion_pointer(_cal_sym, _ppvar, 3, 1);
   nrn_update_ion_pointer(_cat_sym, _ppvar, 4, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 5, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 6, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 7, 4);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  Roff = Roff0;
  Ron = Ron0;
  thresh = thresh0;
  weight = weight0;
 {
   Rinf = Cmax * Alpha / ( Cmax * Alpha + Beta ) ;
   Rtau = 1.0 / ( Cmax * Alpha + Beta ) ;
   synon = 0.0 ;
   weight = w0 ;
   threshf = KD1 ;
   thresh = threshf ;
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
  cai = _ion_cai;
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   B = mgblock ( _threadargscomma_ v ) ;
   g = ( Ron + Roff ) * gmax * B ;
   iNMDA = g * ( v - Erev ) ;
   ica = 0.175 * iNMDA ;
   iNMDA = 0.825 * iNMDA ;
   kernel = supra ( _threadargscomma_ dopamine , weight , ca_nmdai , cali , cai , g , thresh ) ;
   weight = weight + supra ( _threadargscomma_ dopamine , weight , ca_nmdai , cali , cai , g , thresh ) ;
   thresh = thresh + sTresh ( _threadargscomma_ dopamine , thresh , conc0 ) ;
   lthresh_LTP = thresh ;
   hthresh_LTP = thresh ;
   lthresh_LTD = thresh ;
   ca_nmdai_max = conc0 ;
   cali_max = mltype ;
   }
 _current += iNMDA;
 _current += ica;

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
  cai = _ion_cai;
 _g = _nrn_current(_v + .001);
 	{ double _dica;
  _dica = ica;
 _rhs = _nrn_current(_v);
  _ion_dicadv += (_dica - ica)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica * 1.e2/ (_nd_area);
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
  cai = _ion_cai;
 { error =  release();
 if(error){fprintf(stderr,"at line 100 in file adaptive_shom_NMDA.mod:\n	SOLVE release METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
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
static const char* nmodl_filename = "/mnt/data/Spillover_2/mod/adaptive_shom_NMDA.mod";
static const char* nmodl_file_text = 
  "TITLE simple NMDA receptors\n"
  "NEURON {\n"
  "	POINT_PROCESS adaptive_shom_NMDA\n"
  "	RANGE g, Alpha, Beta, Erev, gmax, Cdur, iNMDA,mg, Cmax, eta, alpha, nmda_ca_fraction\n"
  "	RANGE w0, threshf, synon, flagx, conc0, mltypeMin, mltype, kernel, kernel_LTD, lthresh_LTP, hthresh_LTP, lthresh_LTD\n"
  "	RANGE ca_nmdai_max, cali_max, cati_max, cai_max\n"
  "	RANGE learning_rate_w_LTD, learning_rate_w_LTP, learning_rate_thresh_LTD, learning_rate_thresh_LTP, learning_rate_thresh_KD_LTD, KD1, KD2, KD_LTD, n1, n2, n_LTD\n"
  "	NONSPECIFIC_CURRENT iNMDA\n"
  "	POINTER dopamine, stimulus_flag\n"
  "	USEION ca_nmda READ ca_nmdai VALENCE 2	\n"
  "	USEION cal READ cali VALENCE 2\n"
  "	USEION cat READ cati VALENCE 2\n"
  "	USEION ca READ cai WRITE ica VALENCE 2\n"
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
  "	Cdur      = 1  (ms)		: transmitter duration (rising phase)\n"
  "	Alpha	= 4 (/ms /mM)	: forward (binding) rate (4)\n"
  "	Beta 	= 0.01   (/ms)   : backward (unbinding) rate\n"
  "	Erev	= 15	 (mV)		: reversal potential\n"
  "        mg   = 0      (mM)           : external magnesium concentration\n"
  "        eta = 0  :change in the code\n"
  "        alpha = 0 (/mV)\n"
  "	gmax = 0   (uS)\n"
  "        w0 = 0\n"
  "        conc0=0 (mM)\n"
  "        flagx=0 (1)\n"
  "    	mltype=0 (mM)\n"
  "    	mltypeMin=0.00015 (mM)\n"
  "    	learning_rate_w_LTD=0 (1)\n"
  "    	learning_rate_w_LTP=0 (1)\n"
  "    	learning_rate_thresh_LTD=0 (1)\n"
  "    	learning_rate_thresh_LTP=0 (1)\n"
  "    	learning_rate_thresh_KD_LTD = 0\n"
  "    	threshf=0 (mM)    :change in the code\n"
  "    	threshltp=0.02(mM)\n"
  "    	f=0   (ms)\n"
  "    	conc   (mM)\n"
  "	n1 = 5\n"
  "	n2 = 5\n"
  "	KD1 = 1\n"
  "	KD2 = 1\n"
  "	KD_LTD = 1\n"
  "	n_LTD = 1\n"
  "	nmda_ca_fraction = 0.175\n"
  "	kernel_LTD = 0\n"
  "        cai_max = 0\n"
  "	ca_nmdai_max = 0\n"
  "	cali_max = 0\n"
  "	cati_max = 0\n"
  "\n"
  "\n"
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
  "	ica        (nA)\n"
  "	dopamine\n"
  "        cali            (mM)\n"
  "	ca_nmdai        (mM)\n"
  "	cai  (mM)\n"
  "	cati\n"
  "	stimulus_flag\n"
  "	kernel\n"
  "	lthresh_LTP\n"
  "	hthresh_LTP\n"
  "	lthresh_LTD\n"
  "	\n"
  "}\n"
  "\n"
  "STATE {Ron Roff weight thresh}\n"
  "\n"
  "INITIAL {\n"
  "	Rinf = Cmax*Alpha / (Cmax*Alpha + Beta)\n"
  "	Rtau = 1 / (Cmax*Alpha + Beta)\n"
  "	synon = 0\n"
  "	weight = w0\n"
  "	threshf = KD1\n"
  "        thresh = threshf\n"
  "   \n"
  "\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE release METHOD cnexp\n"
  "    B = mgblock(v)\n"
  "	g = (Ron + Roff)* gmax*B\n"
  "	iNMDA = g*(v - Erev)\n"
  "    ica = 0.175*iNMDA   :(5-10 times more permeable to Ca++ than Na+ or K+, Ascher and Nowak, 1988)\n"
  "    iNMDA = 0.825*iNMDA\n"
  "    kernel = supra(dopamine,weight,ca_nmdai,cali,cai,g,thresh)\n"
  "    weight=weight+supra(dopamine,weight,ca_nmdai,cali,cai,g,thresh)\n"
  "    thresh=thresh+sTresh(dopamine,thresh,conc0)\n"
  "    lthresh_LTP = thresh\n"
  "    hthresh_LTP = thresh\n"
  "    lthresh_LTD = thresh\n"
  "    ca_nmdai_max = conc0\n"
  "    cali_max = mltype\n"
  "   \n"
  "}\n"
  "\n"
  "DERIVATIVE release {\n"
  "	Ron' = (synon*Rinf - Ron)/Rtau\n"
  "	Roff' = -Beta*Roff\n"
  "	\n"
  "\n"
  "\n"
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
  "NET_RECEIVE(dummy, on, nspike, r0, t0 (ms)) {\n"
  "	: flag is an implicit argument of NET_RECEIVE and  normally 0\n"
  "        if (flag == 0) { : a spike, so turn on if not already in a Cdur pulse\n"
  "		nspike = nspike + 1\n"
  "		if (!on) {\n"
  "			r0 = r0*exp(-Beta*(t - t0))\n"
  "			t0 = t\n"
  "			on = 1\n"
  "			synon = synon + weight\n"
  "			state_discontinuity(Ron, Ron + r0)\n"
  "			state_discontinuity(Roff, Roff - r0)\n"
  "		}\n"
  ":		 come again in Cdur with flag = current value of nspike\n"
  "		net_send(Cdur, nspike)\n"
  "       }\n"
  "	if (flag == nspike) { : if this associated with last spike then turn off\n"
  "		r0 = weight*Rinf + (r0 - weight*Rinf)*exp(-(t - t0)/Rtau)\n"
  "		t0 = t\n"
  "		synon = synon - weight\n"
  "		state_discontinuity(Ron, Ron - r0)\n"
  "		state_discontinuity(Roff, Roff + r0)\n"
  "		on = 0\n"
  "	}\n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION supra(dopam,we(uS),conc(mM),calic(mM),cai(mM),g_p(uS),tresh(mM))(uS/ms) {\n"
  "    UNITSOFF\n"
  "    \n"
  "    \n"
  "    if (g_p < 1e-7) {\n"
  "       conc0=0\n"
  "       mltype=0\n"
  "    } else {\n"
  "    \n"
  "       if (conc>conc0 ){\n"
  "          conc0=conc\n"
  "       }\n"
  "       if (calic>mltype ){\n"
  "          mltype=calic\n"
  "       }\n"
  "    }\n"
  "    \n"
  "	supra =((learning_rate_w_LTD*we*mltype*funcCalMin(mltype)*(dopam-1))+(learning_rate_w_LTP*funcCal(conc0,tresh)*(dopam+1)))*trap(g_p)*dopam*dopam\n"
  "	\n"
  "	UNITSON    \n"
  "}\n"
  "FUNCTION sTresh(dopam,tresh(mM),conc0)(uM/ms) {\n"
  "    UNITSOFF\n"
  "    sTresh =((-learning_rate_thresh_LTD*funcCalt(conc0,tresh)*(dopam-1))-learning_rate_thresh_LTP*(funcCalt(conc0,tresh)*(dopam+1)))*trap(g)*dopam*dopam\n"
  "	UNITSON    \n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION trap(g_p(uS))() {\n"
  "    UNITSOFF\n"
  "	if (g_p < 1e-7) {\n"
  "		trap = 0\n"
  "	} else {\n"
  "	    trap = 1\n"
  "        }\n"
  "    UNITSON\n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION funcCal(calnm(mM),tresh(mM))() {\n"
  "    UNITSOFF\n"
  "    funcCal= (1-(1 / (1 + exp((-1000*calnm + (1000*tresh))/1))))*(1 / (1 + exp((-1000*calnm+(1000*tresh))/1))) \n"
  "    UNITSON\n"
  "}\n"
  "FUNCTION funcCalt(calnm(mM),tresh(mM))() {\n"
  "    UNITSOFF\n"
  "    funcCalt= (1-(1 / (1 + exp((-1000*calnm + (1000*tresh))/3))))*(1 / (1 + exp((-1000*calnm+(1000*tresh))/3)))\n"
  "    UNITSON\n"
  "}\n"
  "FUNCTION funcCalteg(calnm(mM),tresh(mM))() {\n"
  "    UNITSOFF\n"
  "    funcCalteg= (-1 / (1 + exp((-1000*calnm+(1000*tresh))/2.5))) \n"
  "    UNITSON\n"
  "}\n"
  "FUNCTION funcCalMin(calnm(mM))() {\n"
  "    UNITSOFF\n"
  "    funcCalMin=1 / (1 + exp((-Norm(calnm)+7)/1))\n"
  "    UNITSON\n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION Norm(calnm(mM))() {\n"
  "    UNITSOFF\n"
  "    Norm= 100000*calnm\n"
  "    UNITSON\n"
  "}\n"
  "FUNCTION funcCalMin1(calnm(mM))() {\n"
  "    UNITSOFF\n"
  "    funcCalMin1= 1 / (1 + exp((-100000*calnm+(100000*mltypeMin))/1)) \n"
  "    UNITSON\n"
  "}\n"
  ;
#endif
