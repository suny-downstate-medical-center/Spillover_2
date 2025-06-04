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
 
#define nrn_init _nrn_init__adaptive_glutamate_cshom
#define _nrn_initial _nrn_initial__adaptive_glutamate_cshom
#define nrn_cur _nrn_cur__adaptive_glutamate_cshom
#define _nrn_current _nrn_current__adaptive_glutamate_cshom
#define nrn_jacob _nrn_jacob__adaptive_glutamate_cshom
#define nrn_state _nrn_state__adaptive_glutamate_cshom
#define _net_receive _net_receive__adaptive_glutamate_cshom 
#define state state__adaptive_glutamate_cshom 
 
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
#define erev_ampa _p[0]
#define erev_ampa_columnindex 0
#define erev_nmda _p[1]
#define erev_nmda_columnindex 1
#define tau1_ampa _p[2]
#define tau1_ampa_columnindex 2
#define tau2_ampa _p[3]
#define tau2_ampa_columnindex 3
#define tau1_nmda _p[4]
#define tau1_nmda_columnindex 4
#define tau2_nmda _p[5]
#define tau2_nmda_columnindex 5
#define mg _p[6]
#define mg_columnindex 6
#define alpha _p[7]
#define alpha_columnindex 7
#define q _p[8]
#define q_columnindex 8
#define eta _p[9]
#define eta_columnindex 9
#define NMDA_AMPA_ratio _p[10]
#define NMDA_AMPA_ratio_columnindex 10
#define w0 _p[11]
#define w0_columnindex 11
#define learning_rate_w_LTP _p[12]
#define learning_rate_w_LTP_columnindex 12
#define learning_rate_w_LTD _p[13]
#define learning_rate_w_LTD_columnindex 13
#define thresh_LTP_0 _p[14]
#define thresh_LTP_0_columnindex 14
#define thresh_LTD_0 _p[15]
#define thresh_LTD_0_columnindex 15
#define hthresh_LTP_0 _p[16]
#define hthresh_LTP_0_columnindex 16
#define hthresh_max _p[17]
#define hthresh_max_columnindex 17
#define lthresh_LTP_min _p[18]
#define lthresh_LTP_min_columnindex 18
#define delta _p[19]
#define delta_columnindex 19
#define width _p[20]
#define width_columnindex 20
#define hthresh_LTP_const _p[21]
#define hthresh_LTP_const_columnindex 21
#define learning_rate_thresh_LTP _p[22]
#define learning_rate_thresh_LTP_columnindex 22
#define learning_rate_thresh_LTD _p[23]
#define learning_rate_thresh_LTD_columnindex 23
#define n _p[24]
#define n_columnindex 24
#define steepness_LTP _p[25]
#define steepness_LTP_columnindex 25
#define steepness_LTD _p[26]
#define steepness_LTD_columnindex 26
#define nmda_ca_fraction _p[27]
#define nmda_ca_fraction_columnindex 27
#define ca_nmdai_max _p[28]
#define ca_nmdai_max_columnindex 28
#define cali_max _p[29]
#define cali_max_columnindex 29
#define active_syn_flag _p[30]
#define active_syn_flag_columnindex 30
#define i _p[31]
#define i_columnindex 31
#define g _p[32]
#define g_columnindex 32
#define i_ampa _p[33]
#define i_ampa_columnindex 33
#define i_nmda _p[34]
#define i_nmda_columnindex 34
#define g_ampa _p[35]
#define g_ampa_columnindex 35
#define g_nmda _p[36]
#define g_nmda_columnindex 36
#define I _p[37]
#define I_columnindex 37
#define G _p[38]
#define G_columnindex 38
#define last_dopamine _p[39]
#define last_dopamine_columnindex 39
#define weight _p[40]
#define weight_columnindex 40
#define lthresh_LTP _p[41]
#define lthresh_LTP_columnindex 41
#define lthresh_LTD _p[42]
#define lthresh_LTD_columnindex 42
#define hthresh_LTP _p[43]
#define hthresh_LTP_columnindex 43
#define delta_LTP _p[44]
#define delta_LTP_columnindex 44
#define A _p[45]
#define A_columnindex 45
#define B _p[46]
#define B_columnindex 46
#define C _p[47]
#define C_columnindex 47
#define D _p[48]
#define D_columnindex 48
#define factor_nmda _p[49]
#define factor_nmda_columnindex 49
#define factor_ampa _p[50]
#define factor_ampa_columnindex 50
#define block _p[51]
#define block_columnindex 51
#define ica_nmda _p[52]
#define ica_nmda_columnindex 52
#define ca_nmdai _p[53]
#define ca_nmdai_columnindex 53
#define cali _p[54]
#define cali_columnindex 54
#define DA _p[55]
#define DA_columnindex 55
#define DB _p[56]
#define DB_columnindex 56
#define DC _p[57]
#define DC_columnindex 57
#define DD _p[58]
#define DD_columnindex 58
#define _g _p[59]
#define _g_columnindex 59
#define _tsav _p[60]
#define _tsav_columnindex 60
#define _nd_area  *_ppvar[0]._pval
#define _ion_ca_nmdai	*_ppvar[2]._pval
#define _ion_ica_nmda	*_ppvar[3]._pval
#define _ion_dica_nmdadv	*_ppvar[4]._pval
#define _ion_cali	*_ppvar[5]._pval
#define dopamine	*_ppvar[6]._pval
#define _p_dopamine	_ppvar[6]._pval
#define stimulus_flag	*_ppvar[7]._pval
#define _p_stimulus_flag	_ppvar[7]._pval
 
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
 static int hoc_nrnpointerindex =  6;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_MgBlock(void*);
 static double _hoc_hthresh(void*);
 static double _hoc_lthresh(void*);
 static double _hoc_min(void*);
 static double _hoc_max(void*);
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
 "MgBlock", _hoc_MgBlock,
 "hthresh", _hoc_hthresh,
 "lthresh", _hoc_lthresh,
 "min", _hoc_min,
 "max", _hoc_max,
 "reset_max", _hoc_reset_max,
 "sigmoidal", _hoc_sigmoidal,
 0, 0
};
#define MgBlock MgBlock_adaptive_glutamate_cshom
#define hthresh hthresh_adaptive_glutamate_cshom
#define lthresh lthresh_adaptive_glutamate_cshom
#define min min_adaptive_glutamate_cshom
#define max max_adaptive_glutamate_cshom
#define reset_max reset_max_adaptive_glutamate_cshom
#define sigmoidal sigmoidal_adaptive_glutamate_cshom
 extern double MgBlock( );
 extern double hthresh( double , double , double );
 extern double lthresh( double , double , double );
 extern double min( double , double );
 extern double max( double , double );
 extern double reset_max( );
 extern double sigmoidal( double , double , double );
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "erev_ampa", "mV",
 "erev_nmda", "mV",
 "tau1_ampa", "ms",
 "tau2_ampa", "ms",
 "tau1_nmda", "ms",
 "tau2_nmda", "ms",
 "mg", "mM",
 "A", "uS",
 "B", "uS",
 "C", "uS",
 "D", "uS",
 "i", "nA",
 "g", "uS",
 0,0
};
 static double A0 = 0;
 static double B0 = 0;
 static double C0 = 0;
 static double D0 = 0;
 static double delta_t = 0.01;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
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
 
#define _cvode_ieq _ppvar[8]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"adaptive_glutamate_cshom",
 "erev_ampa",
 "erev_nmda",
 "tau1_ampa",
 "tau2_ampa",
 "tau1_nmda",
 "tau2_nmda",
 "mg",
 "alpha",
 "q",
 "eta",
 "NMDA_AMPA_ratio",
 "w0",
 "learning_rate_w_LTP",
 "learning_rate_w_LTD",
 "thresh_LTP_0",
 "thresh_LTD_0",
 "hthresh_LTP_0",
 "hthresh_max",
 "lthresh_LTP_min",
 "delta",
 "width",
 "hthresh_LTP_const",
 "learning_rate_thresh_LTP",
 "learning_rate_thresh_LTD",
 "n",
 "steepness_LTP",
 "steepness_LTD",
 "nmda_ca_fraction",
 "ca_nmdai_max",
 "cali_max",
 "active_syn_flag",
 0,
 "i",
 "g",
 "i_ampa",
 "i_nmda",
 "g_ampa",
 "g_nmda",
 "I",
 "G",
 "last_dopamine",
 "weight",
 "lthresh_LTP",
 "lthresh_LTD",
 "hthresh_LTP",
 "delta_LTP",
 0,
 "A",
 "B",
 "C",
 "D",
 0,
 "dopamine",
 "stimulus_flag",
 0};
 static Symbol* _ca_nmda_sym;
 static Symbol* _cal_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 61, _prop);
 	/*initialize range parameters*/
 	erev_ampa = 0;
 	erev_nmda = 15;
 	tau1_ampa = 1.9;
 	tau2_ampa = 4.8;
 	tau1_nmda = 5.52;
 	tau2_nmda = 231;
 	mg = 1;
 	alpha = 0.062;
 	q = 2;
 	eta = 18;
 	NMDA_AMPA_ratio = 1;
 	w0 = 0.01;
 	learning_rate_w_LTP = 0.01;
 	learning_rate_w_LTD = 0.01;
 	thresh_LTP_0 = 0.07;
 	thresh_LTD_0 = 0.005;
 	hthresh_LTP_0 = 0.5;
 	hthresh_max = 2;
 	lthresh_LTP_min = 0.055;
 	delta = 0.65;
 	width = 0.25;
 	hthresh_LTP_const = 0.05;
 	learning_rate_thresh_LTP = 0.005;
 	learning_rate_thresh_LTD = 0.005;
 	n = 4;
 	steepness_LTP = 0.25;
 	steepness_LTD = 2.5;
 	nmda_ca_fraction = 0.15;
 	ca_nmdai_max = 0;
 	cali_max = 0;
 	active_syn_flag = 1e-06;
  }
 	_prop->param = _p;
 	_prop->param_size = 61;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 9, _prop);
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
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _net_receive(Point_process*, double*, double);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _adaptive_glutamate_cshom_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca_nmda", 2.0);
 	ion_reg("cal", 2.0);
 	_ca_nmda_sym = hoc_lookup("ca_nmda_ion");
 	_cal_sym = hoc_lookup("cal_ion");
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
  hoc_register_prop_size(_mechtype, 61, 9);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "pointer");
  hoc_register_dparam_semantics(_mechtype, 7, "pointer");
  hoc_register_dparam_semantics(_mechtype, 8, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 adaptive_glutamate_cshom /home/shadeform/Spillover_2/mod/adaptive_glutamate_cshom.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[4], _dlist1[4];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   DA = - A / tau1_nmda * q ;
   DB = - B / tau2_nmda * q ;
   DC = - C / tau1_ampa ;
   DD = - D / tau2_ampa ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 DA = DA  / (1. - dt*( ( ( - 1.0 ) / tau1_nmda )*( q ) )) ;
 DB = DB  / (1. - dt*( ( ( - 1.0 ) / tau2_nmda )*( q ) )) ;
 DC = DC  / (1. - dt*( ( - 1.0 ) / tau1_ampa )) ;
 DD = DD  / (1. - dt*( ( - 1.0 ) / tau2_ampa )) ;
  return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
    A = A + (1. - exp(dt*(( ( - 1.0 ) / tau1_nmda )*( q ))))*(- ( 0.0 ) / ( ( ( - 1.0 ) / tau1_nmda )*( q ) ) - A) ;
    B = B + (1. - exp(dt*(( ( - 1.0 ) / tau2_nmda )*( q ))))*(- ( 0.0 ) / ( ( ( - 1.0 ) / tau2_nmda )*( q ) ) - B) ;
    C = C + (1. - exp(dt*(( - 1.0 ) / tau1_ampa)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau1_ampa ) - C) ;
    D = D + (1. - exp(dt*(( - 1.0 ) / tau2_ampa)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau2_ampa ) - D) ;
   }
  return 0;
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{    _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(Object*); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
   active_syn_flag = 1.0 ;
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A;
    double __primary = (A + factor_nmda) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( ( - 1.0 ) / tau1_nmda )*( q ) ) ) )*( - ( 0.0 ) / ( ( ( - 1.0 ) / tau1_nmda )*( q ) ) - __primary );
    A += __primary;
  } else {
 A = A + factor_nmda ;
     }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B;
    double __primary = (B + factor_nmda) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( ( - 1.0 ) / tau2_nmda )*( q ) ) ) )*( - ( 0.0 ) / ( ( ( - 1.0 ) / tau2_nmda )*( q ) ) - __primary );
    B += __primary;
  } else {
 B = B + factor_nmda ;
     }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = C;
    double __primary = (C + factor_ampa) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau1_ampa ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau1_ampa ) - __primary );
    C += __primary;
  } else {
 C = C + factor_ampa ;
     }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = D;
    double __primary = (D + factor_ampa) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau2_ampa ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau2_ampa ) - __primary );
    D += __primary;
  } else {
 D = D + factor_ampa ;
     }
 } }
 
double MgBlock (  ) {
   double _lMgBlock;
 _lMgBlock = 1.0 / ( 1.0 + mg * eta * exp ( - alpha * v ) ) ;
   
return _lMgBlock;
 }
 
static double _hoc_MgBlock(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  MgBlock (  );
 return(_r);
}
 
double lthresh (  double _lconc , double _lKD , double _lsteepness ) {
   double _llthresh;
 _llthresh = sigmoidal ( _threadargscomma_ _lconc , _lKD , _lsteepness ) ;
   
return _llthresh;
 }
 
static double _hoc_lthresh(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  lthresh (  *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double hthresh (  double _lconc , double _lKD , double _lsteepness ) {
   double _lhthresh;
 _lhthresh = 1.0 - sigmoidal ( _threadargscomma_ _lconc , _lKD , _lsteepness ) ;
   
return _lhthresh;
 }
 
static double _hoc_hthresh(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  hthresh (  *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double reset_max (  ) {
   double _lreset_max;
 ca_nmdai_max = 0.0 ;
   cali_max = 0.0 ;
   active_syn_flag = 0.0 ;
   
return _lreset_max;
 }
 
static double _hoc_reset_max(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  reset_max (  );
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
 
static int _ode_count(int _type){ return 4;}
 
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
     _ode_spec1 ();
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 4; ++_i) {
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
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 2, 1);
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 3, 3);
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 4, 4);
   nrn_update_ion_pointer(_cal_sym, _ppvar, 5, 1);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  A = A0;
  B = B0;
  C = C0;
  D = D0;
 {
   double _ltp ;
 if ( tau1_nmda / tau2_nmda > .9999 ) {
     tau1_nmda = .9999 * tau2_nmda ;
     }
   if ( tau1_ampa / tau2_ampa > .9999 ) {
     tau1_ampa = .9999 * tau2_ampa ;
     }
   A = 0.0 ;
   B = 0.0 ;
   _ltp = ( tau1_nmda * tau2_nmda ) / ( tau2_nmda - tau1_nmda ) * log ( tau2_nmda / tau1_nmda ) ;
   factor_nmda = - exp ( - _ltp / tau1_nmda ) + exp ( - _ltp / tau2_nmda ) ;
   factor_nmda = 1.0 / factor_nmda ;
   C = 0.0 ;
   D = 0.0 ;
   _ltp = ( tau1_ampa * tau2_ampa ) / ( tau2_ampa - tau1_ampa ) * log ( tau2_ampa / tau1_ampa ) ;
   factor_ampa = - exp ( - _ltp / tau1_ampa ) + exp ( - _ltp / tau2_ampa ) ;
   factor_ampa = 1.0 / factor_ampa ;
   weight = w0 ;
   lthresh_LTP = thresh_LTP_0 ;
   lthresh_LTD = thresh_LTD_0 ;
   hthresh_LTP = hthresh_LTP_0 ;
   active_syn_flag = 0.0 ;
   last_dopamine = 0.0 ;
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
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   g_nmda = ( B - A ) * weight * NMDA_AMPA_ratio ;
   block = MgBlock ( _threadargs_ ) ;
   i_nmda = g_nmda * ( v - erev_nmda ) * block ;
   ica_nmda = nmda_ca_fraction * i_nmda ;
   i_nmda = ( 1.0 - nmda_ca_fraction ) * i_nmda ;
   g_ampa = ( D - C ) * weight ;
   i_ampa = g_ampa * ( v - erev_ampa ) ;
   G = g_ampa + g_nmda ;
   I = i_ampa ;
   i = I ;
   if ( stimulus_flag  == 1.0 ) {
     ca_nmdai_max = max ( _threadargscomma_ ca_nmdai , ca_nmdai_max ) ;
     cali_max = max ( _threadargscomma_ cali , cali_max ) ;
     last_dopamine = dopamine ;
     }
   else {
     if ( last_dopamine  == 1.0  && active_syn_flag  == 1.0 ) {
       delta_LTP = sigmoidal ( _threadargscomma_ ca_nmdai_max , lthresh_LTP , steepness_LTP ) * ( 1.0 - sigmoidal ( _threadargscomma_ ca_nmdai_max , lthresh_LTP , steepness_LTP ) ) ;
       weight = weight + learning_rate_w_LTP * delta_LTP ;
       lthresh_LTP = lthresh_LTP + learning_rate_thresh_LTP * delta_LTP * ( 1.0 - 2.0 * sigmoidal ( _threadargscomma_ ca_nmdai_max , lthresh_LTP , steepness_LTP ) ) ;
       }
     else if ( last_dopamine  == - 1.0  && active_syn_flag  == 1.0 ) {
       weight = weight - learning_rate_w_LTD * cali_max * sigmoidal ( _threadargscomma_ cali_max , lthresh_LTD , steepness_LTD ) * weight ;
       lthresh_LTP = lthresh_LTP - learning_rate_thresh_LTP * ( lthresh_LTP - lthresh_LTP_min ) ;
       }
     last_dopamine = dopamine ;
     reset_max ( _threadargs_ ) ;
     }
   }
 _current += i;
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
 { error =  state();
 if(error){fprintf(stderr,"at line 175 in file adaptive_glutamate_cshom.mod:\n	: NMDA\n"); nrn_complain(_p); abort_run(error);}
 } }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = A_columnindex;  _dlist1[0] = DA_columnindex;
 _slist1[1] = B_columnindex;  _dlist1[1] = DB_columnindex;
 _slist1[2] = C_columnindex;  _dlist1[2] = DC_columnindex;
 _slist1[3] = D_columnindex;  _dlist1[3] = DD_columnindex;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "/home/shadeform/Spillover_2/mod/adaptive_glutamate_cshom.mod";
static const char* nmodl_file_text = 
  "COMMENT\n"
  "Updated Exp2Syn synapse with Mg-blocked nmda channel.\n"
  "\n"
  "Defaul values of parameters (time constants etc) set to match synaptic channels in \n"
  "striatal medium spiny neurons (Du et al., 2017; Chapman et al., 2003; Ding et al., 2008).\n"
  "\n"
  "Robert . Lindroos @ ki . se\n"
  "\n"
  "original comment:\n"
  "________________\n"
  "Two state kinetic scheme synapse described by rise time tau1,\n"
  "and decay time constant tau2. The normalized peak condunductance is 1.\n"
  "Decay time MUST be greater than rise time.\n"
  "\n"
  "The solution of A->G->bath with rate constants 1/tau1 and 1/tau2 is\n"
  " A = a*exp(-t/tau1) and\n"
  " G = a*tau2/(tau2-tau1)*(-exp(-t/tau1) + exp(-t/tau2))\n"
  "	where tau1 < tau2\n"
  "\n"
  "If tau2-tau1 -> 0 then we have a alphasynapse.\n"
  "and if tau1 -> 0 then we have just single exponential decay.\n"
  "\n"
  "The factor is evaluated in the\n"
  "initial block such that an event of weight 1 generates a\n"
  "peak conductance of 1.\n"
  "\n"
  "Because the solution is a sum of exponentials, the\n"
  "coupled equations can be solved as a pair of independent equations\n"
  "by the more efficient cnexp method.\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS adaptive_glutamate_cshom\n"
  "	RANGE tau1_ampa, tau2_ampa, tau1_nmda, tau2_nmda\n"
  "	RANGE erev_ampa, erev_nmda, g, i\n"
  "	NONSPECIFIC_CURRENT i\n"
  "	\n"
  "	RANGE i_ampa, i_nmda, g_ampa, g_nmda, I, G, mg, q, alpha, eta\n"
  "	RANGE w0, NMDA_AMPA_ratio\n"
  "	RANGE weight, lthresh_LTP, lthresh_LTD, hthresh_LTP, last_dopamine, lthresh_LTP_min\n"
  "        RANGE hthresh_LTP_const, hthresh_LTP_0, hthresh_max, n, delta, width, steepness_LTP, steepness_LTD\n"
  "	RANGE learning_rate_w_LTP, learning_rate_w_LTD, thresh_LTP_0,  learning_rate_thresh_LTP, thresh_LTD_0, learning_rate_thresh_LTD, hthresh_LTP_0\n"
  "	RANGE ca_nmdai_max, cali_max, active_syn_flag, nmda_ca_fraction, delta_LTP\n"
  "	POINTER dopamine, stimulus_flag\n"
  "	USEION ca_nmda READ ca_nmdai WRITE ica_nmda VALENCE 2	\n"
  "	USEION cal READ cali VALENCE 2\n"
  "}\n"
  "\n"
  "\n"
  "UNITS {\n"
  "	(nA) = (nanoamp)\n"
  "	(mV) = (millivolt)\n"
  "	(uS) = (microsiemens)\n"
  "}\n"
  "\n"
  "\n"
  "PARAMETER {\n"
  "	erev_ampa        = 0.0       (mV)\n"
  "	erev_nmda 	 = 15.0 (mV)\n"
  "	tau1_ampa   = 1.9       (ms)\n"
  "    	tau2_ampa   = 4.8       (ms)  : tau2 > tau1\n"
  "    	tau1_nmda   = 5.52      (ms)  : old value was 5.63\n"
  "    	tau2_nmda   = 231       (ms)  : tau2 > tau1\n"
  "    \n"
  "    	mg          = 1         (mM)\n"
  "    	alpha       = 0.062\n"
  "    	q           = 2\n"
  "    	eta 	= 18\n"
  "	NMDA_AMPA_ratio = 1\n"
  "	w0 = 0.01\n"
  "\n"
  "    	learning_rate_w_LTP = 0.01\n"
  "	learning_rate_w_LTD = 0.01\n"
  "	\n"
  "	thresh_LTP_0 = 0.07\n"
  "	thresh_LTD_0 = 0.005\n"
  "	hthresh_LTP_0 = 0.5\n"
  "	hthresh_max = 2.0\n"
  "        lthresh_LTP_min = 0.055\n"
  "\n"
  "	delta = 0.65\n"
  "        width = 0.25\n"
  "        hthresh_LTP_const = 0.05\n"
  "	learning_rate_thresh_LTP = 0.005\n"
  "	learning_rate_thresh_LTD = 0.005\n"
  "	n = 4 : Hill coefficient\n"
  "        steepness_LTP = 0.25\n"
  "	steepness_LTD = 2.5\n"
  "        nmda_ca_fraction = 0.15\n"
  "\n"
  "	ca_nmdai_max = 0\n"
  "	cali_max = 0\n"
  "	active_syn_flag = 1e-6\n"
  "}\n"
  "\n"
  "\n"
  "ASSIGNED {\n"
  "	v (mV)\n"
  "	i (nA)\n"
  "	g (uS)\n"
  "	factor_nmda\n"
  "	factor_ampa\n"
  "	i_ampa\n"
  "	i_nmda\n"
  "	g_ampa\n"
  "	g_nmda\n"
  "	block\n"
  "	I\n"
  "	G\n"
  "\n"
  "	stimulus_flag\n"
  "	dopamine\n"
  "        last_dopamine\n"
  "        ica_nmda (nA)\n"
  "	ca_nmdai (mM)\n"
  "	cali (mM)\n"
  "\n"
  "	weight\n"
  "	lthresh_LTP\n"
  "	lthresh_LTD\n"
  "	hthresh_LTP\n"
  "	delta_LTP\n"
  "}\n"
  "\n"
  "\n"
  "STATE {\n"
  "	A (uS)\n"
  "	B (uS)\n"
  "	C (uS)\n"
  "	D (uS)\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "INITIAL {\n"
  "	LOCAL tp\n"
  "	if (tau1_nmda/tau2_nmda > .9999) {\n"
  "		tau1_nmda = .9999*tau2_nmda\n"
  "	}\n"
  "	if (tau1_ampa/tau2_ampa > .9999) {\n"
  "		tau1_ampa = .9999*tau2_ampa\n"
  "	}\n"
  "	\n"
  "	: NMDA\n"
  "	A           = 0\n"
  "	B           = 0\n"
  "	tp          = (tau1_nmda*tau2_nmda)/(tau2_nmda - tau1_nmda) * log(tau2_nmda/tau1_nmda)\n"
  "	factor_nmda = -exp(-tp/tau1_nmda) + exp(-tp/tau2_nmda)\n"
  "	factor_nmda = 1/factor_nmda\n"
  "	\n"
  "	: AMPA\n"
  "	C           = 0\n"
  "	D           = 0\n"
  "	tp          = (tau1_ampa*tau2_ampa)/(tau2_ampa - tau1_ampa) * log(tau2_ampa/tau1_ampa)\n"
  "	factor_ampa = -exp(-tp/tau1_ampa) + exp(-tp/tau2_ampa)\n"
  "	factor_ampa = 1/factor_ampa\n"
  "	\n"
  "	weight = w0 \n"
  "	lthresh_LTP = thresh_LTP_0\n"
  "	lthresh_LTD = thresh_LTD_0\n"
  "	hthresh_LTP = hthresh_LTP_0\n"
  "	active_syn_flag = 0\n"
  "        last_dopamine = 0\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE state METHOD cnexp\n"
  "	\n"
  "	: NMDA\n"
  "	g_nmda = (B - A)*weight*NMDA_AMPA_ratio\n"
  "	block  = MgBlock()\n"
  "	i_nmda = g_nmda * (v - erev_nmda) * block\n"
  "        ica_nmda = nmda_ca_fraction*i_nmda\n"
  "        i_nmda = (1 - nmda_ca_fraction)*i_nmda\n"
  "	\n"
  "	: AMPA\n"
  "	g_ampa = (D - C)*weight\n"
  "	i_ampa = g_ampa * (v - erev_ampa)\n"
  "	\n"
  "	: total current\n"
  "	G = g_ampa + g_nmda\n"
  "	I = i_ampa\n"
  "        i = I\n"
  "\n"
  "        if (stimulus_flag == 1) {\n"
  "        	ca_nmdai_max = max(ca_nmdai, ca_nmdai_max)\n"
  "        	cali_max = max(cali, cali_max)\n"
  "		last_dopamine = dopamine\n"
  "        } else {\n"
  "	  if (last_dopamine == 1 && active_syn_flag == 1) {\n"
  "                 \n"
  "                 delta_LTP = sigmoidal(ca_nmdai_max, lthresh_LTP, steepness_LTP) * (1 - sigmoidal(ca_nmdai_max, lthresh_LTP, steepness_LTP))\n"
  "		  weight = weight + learning_rate_w_LTP * delta_LTP\n"
  "		  lthresh_LTP = lthresh_LTP + learning_rate_thresh_LTP * delta_LTP * (1 - 2*sigmoidal(ca_nmdai_max, lthresh_LTP, steepness_LTP))\n"
  "          \n"
  "          } else if (last_dopamine == -1 && active_syn_flag == 1) {\n"
  "\n"
  "		  weight = weight - learning_rate_w_LTD * cali_max * sigmoidal(cali_max, lthresh_LTD, steepness_LTD) * weight \n"
  "		  lthresh_LTP = lthresh_LTP - learning_rate_thresh_LTP * (lthresh_LTP - lthresh_LTP_min)\n"
  "\n"
  "          }\n"
  "          last_dopamine = dopamine		\n"
  "          reset_max()\n"
  "}\n"
  "\n"
  "	:        delta_LTP = lthresh(ca_nmdai_max, lthresh_LTP, steepness_LTP) * hthresh(ca_nmdai_max, lthresh_LTP, steepness_LTP)\n"
  "	:	  weight = weight + learning_rate_w_LTP * lthresh(ca_nmdai_max, lthresh_LTP, steepness_LTP) * hthresh(ca_nmdai_max, hthresh_LTP, steepness_LTP)\n"
  "	:	  lthresh_LTP = lthresh_LTP + learning_rate_thresh_LTP * delta_LTP * (1 - 2*sigmoid(ca_nmdai_max, lthresh_LTP, steepness_LTP))\n"
  "	:	  lthresh_LTP = max(lthresh_LTP, lthresh_LTP_min)\n"
  "		  \n"
  "		  :lthresh_LTP = lthresh_LTP + learning_rate_thresh_LTP * lthresh(ca_nmdai_max, lthresh_LTP, steepness_LTP)* ((ca_nmdai_max-width)* delta - lthresh_LTP)\n"
  "		  :lthresh_LTD = lthresh_LTD + learning_rate_thresh_LTD *lthresh(ca_nmdai_max, lthresh_LTP, steepness_LTP) * (cali_max*delta - lthresh_LTD)\n"
  "		  :hthresh_LTP = lthresh_LTP + width\n"
  "\n"
  "          \n"
  "	:	  weight = weight - learning_rate_w_LTD * cali_max * lthresh(cali_max, lthresh_LTD, steepness_LTD) * weight\n"
  "	:	  lthresh_LTP = lthresh_LTP - learning_rate_thresh_LTP * lthresh(cali_max, lthresh_LTD, steepness_LTD)*(lthresh_LTP - max(ca_nmdai_max, lthresh_LTP_min))\n"
  "	:	  lthresh_LTP = max(lthresh_LTP, lthresh_LTP_min)\n"
  "		  \n"
  "		  :lthresh_LTP = lthresh_LTP - learning_rate_thresh_LTP *lthresh(cali_max, lthresh_LTD, steepness_LTD)*(lthresh_LTP - max(ca_nmdai_max, lthresh_LTP_min))\n"
  "		  :lthresh_LTD = lthresh_LTD - learning_rate_thresh_LTD *lthresh(cali_max, lthresh_LTD, steepness_LTD)*(lthresh_LTD - delta*cali_max)\n"
  "		  :hthresh_LTP = lthresh_LTP + width\n"
  "\n"
  "         \n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "DERIVATIVE state {\n"
  "	A' = -A/tau1_nmda*q\n"
  "	B' = -B/tau2_nmda*q\n"
  "	C' = -C/tau1_ampa\n"
  "	D' = -D/tau2_ampa\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "NET_RECEIVE(dummy (uS)) {\n"
  "	active_syn_flag = 1\n"
  "	\n"
  "	A = A + factor_nmda\n"
  "	B = B + factor_nmda\n"
  "	C = C + factor_ampa\n"
  "	D = D + factor_ampa\n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION MgBlock() {\n"
  "    \n"
  "    MgBlock = 1 / (1 + mg * eta * exp(-alpha * v)  )\n"
  "    \n"
  "}\n"
  "\n"
  "FUNCTION lthresh(conc, KD, steepness) {\n"
  ":    lthresh = conc^n/(KD^n + conc^n)\n"
  "    lthresh = sigmoidal(conc,KD, steepness)  \n"
  "}\n"
  "\n"
  "FUNCTION hthresh(conc, KD, steepness) {\n"
  ":    hthresh = KD^n/(KD^n + conc^n)\n"
  "    hthresh = 1 - sigmoidal(conc, KD, steepness)  \n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION reset_max() {\n"
  "	ca_nmdai_max = 0\n"
  "        cali_max = 0\n"
  "	active_syn_flag = 0\n"
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
  "FUNCTION sigmoidal(x, x_offset, s) {\n"
  "    sigmoidal = 1/(1+exp(-s *(x - x_offset)))\n"
  "}\n"
  ;
#endif
