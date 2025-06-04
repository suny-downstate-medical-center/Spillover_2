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
 
#define nrn_init _nrn_init__adaptive_glutamate_test
#define _nrn_initial _nrn_initial__adaptive_glutamate_test
#define nrn_cur _nrn_cur__adaptive_glutamate_test
#define _nrn_current _nrn_current__adaptive_glutamate_test
#define nrn_jacob _nrn_jacob__adaptive_glutamate_test
#define nrn_state _nrn_state__adaptive_glutamate_test
#define _net_receive _net_receive__adaptive_glutamate_test 
#define state state__adaptive_glutamate_test 
 
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
#define ratio _p[6]
#define ratio_columnindex 6
#define mg _p[7]
#define mg_columnindex 7
#define alpha _p[8]
#define alpha_columnindex 8
#define q _p[9]
#define q_columnindex 9
#define eta _p[10]
#define eta_columnindex 10
#define w_ampa_min _p[11]
#define w_ampa_min_columnindex 11
#define w_ampa_0 _p[12]
#define w_ampa_0_columnindex 12
#define w_ampa_max _p[13]
#define w_ampa_max_columnindex 13
#define w_nmda_0 _p[14]
#define w_nmda_0_columnindex 14
#define w_nmda_plateau _p[15]
#define w_nmda_plateau_columnindex 15
#define glu_thresh1 _p[16]
#define glu_thresh1_columnindex 16
#define glu_thresh2 _p[17]
#define glu_thresh2_columnindex 17
#define thresh_LTP _p[18]
#define thresh_LTP_columnindex 18
#define thresh_LTD _p[19]
#define thresh_LTD_columnindex 19
#define learning_rate _p[20]
#define learning_rate_columnindex 20
#define i _p[21]
#define i_columnindex 21
#define g _p[22]
#define g_columnindex 22
#define i_ampa _p[23]
#define i_ampa_columnindex 23
#define i_nmda _p[24]
#define i_nmda_columnindex 24
#define g_ampa _p[25]
#define g_ampa_columnindex 25
#define g_nmda _p[26]
#define g_nmda_columnindex 26
#define I _p[27]
#define I_columnindex 27
#define G _p[28]
#define G_columnindex 28
#define A _p[29]
#define A_columnindex 29
#define B _p[30]
#define B_columnindex 30
#define C _p[31]
#define C_columnindex 31
#define D _p[32]
#define D_columnindex 32
#define w_nmda _p[33]
#define w_nmda_columnindex 33
#define w_ampa _p[34]
#define w_ampa_columnindex 34
#define factor_nmda _p[35]
#define factor_nmda_columnindex 35
#define factor_ampa _p[36]
#define factor_ampa_columnindex 36
#define block _p[37]
#define block_columnindex 37
#define ica_nmda _p[38]
#define ica_nmda_columnindex 38
#define ca_nmdai _p[39]
#define ca_nmdai_columnindex 39
#define cali _p[40]
#define cali_columnindex 40
#define DA _p[41]
#define DA_columnindex 41
#define DB _p[42]
#define DB_columnindex 42
#define DC _p[43]
#define DC_columnindex 43
#define DD _p[44]
#define DD_columnindex 44
#define Dw_nmda _p[45]
#define Dw_nmda_columnindex 45
#define Dw_ampa _p[46]
#define Dw_ampa_columnindex 46
#define _g _p[47]
#define _g_columnindex 47
#define _tsav _p[48]
#define _tsav_columnindex 48
#define _nd_area  *_ppvar[0]._pval
#define _ion_ca_nmdai	*_ppvar[2]._pval
#define _ion_ica_nmda	*_ppvar[3]._pval
#define _ion_dica_nmdadv	*_ppvar[4]._pval
#define _ion_cali	*_ppvar[5]._pval
#define glu	*_ppvar[6]._pval
#define _p_glu	_ppvar[6]._pval
#define dopamine	*_ppvar[7]._pval
#define _p_dopamine	_ppvar[7]._pval
 
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
 static double _hoc_active_syn(void*);
 static double _hoc_gluind2(void*);
 static double _hoc_gluind1(void*);
 static double _hoc_pind_LTD(void*);
 static double _hoc_pind_LTP(void*);
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
 "MgBlock", _hoc_MgBlock,
 "active_syn", _hoc_active_syn,
 "gluind2", _hoc_gluind2,
 "gluind1", _hoc_gluind1,
 "pind_LTD", _hoc_pind_LTD,
 "pind_LTP", _hoc_pind_LTP,
 "trap", _hoc_trap,
 0, 0
};
#define MgBlock MgBlock_adaptive_glutamate_test
#define active_syn active_syn_adaptive_glutamate_test
#define gluind2 gluind2_adaptive_glutamate_test
#define gluind1 gluind1_adaptive_glutamate_test
#define pind_LTD pind_LTD_adaptive_glutamate_test
#define pind_LTP pind_LTP_adaptive_glutamate_test
#define trap trap_adaptive_glutamate_test
 extern double MgBlock( );
 extern double active_syn( double );
 extern double gluind2( double );
 extern double gluind1( double );
 extern double pind_LTD( double );
 extern double pind_LTP( double );
 extern double trap( double );
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
 "ratio", "1",
 "mg", "mM",
 "w_ampa_min", "uS",
 "w_ampa_0", "uS",
 "w_ampa_max", "uS",
 "w_nmda_0", "uS",
 "w_nmda_plateau", "uS",
 "A", "uS",
 "B", "uS",
 "C", "uS",
 "D", "uS",
 "w_nmda", "uS",
 "w_ampa", "uS",
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
 static double w_ampa0 = 0;
 static double w_nmda0 = 0;
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
"adaptive_glutamate_test",
 "erev_ampa",
 "erev_nmda",
 "tau1_ampa",
 "tau2_ampa",
 "tau1_nmda",
 "tau2_nmda",
 "ratio",
 "mg",
 "alpha",
 "q",
 "eta",
 "w_ampa_min",
 "w_ampa_0",
 "w_ampa_max",
 "w_nmda_0",
 "w_nmda_plateau",
 "glu_thresh1",
 "glu_thresh2",
 "thresh_LTP",
 "thresh_LTD",
 "learning_rate",
 0,
 "i",
 "g",
 "i_ampa",
 "i_nmda",
 "g_ampa",
 "g_nmda",
 "I",
 "G",
 0,
 "A",
 "B",
 "C",
 "D",
 "w_nmda",
 "w_ampa",
 0,
 "glu",
 "dopamine",
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
 	_p = nrn_prop_data_alloc(_mechtype, 49, _prop);
 	/*initialize range parameters*/
 	erev_ampa = 0;
 	erev_nmda = 15;
 	tau1_ampa = 1.9;
 	tau2_ampa = 4.8;
 	tau1_nmda = 5.52;
 	tau2_nmda = 231;
 	ratio = 1;
 	mg = 1;
 	alpha = 0.062;
 	q = 2;
 	eta = 18;
 	w_ampa_min = 0.055;
 	w_ampa_0 = 0.00011;
 	w_ampa_max = 0.000165;
 	w_nmda_0 = 8e-05;
 	w_nmda_plateau = 0.0008;
 	glu_thresh1 = 0.05;
 	glu_thresh2 = 0.05;
 	thresh_LTP = 0.025;
 	thresh_LTD = 0.005;
 	learning_rate = 0.01;
  }
 	_prop->param = _p;
 	_prop->param_size = 49;
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

 void _adaptive_glutamate_test_reg() {
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
  hoc_register_prop_size(_mechtype, 49, 9);
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
 	ivoc_help("help ?1 adaptive_glutamate_test /home/shadeform/Spillover_2/mod/adaptive_glutamate_test.mod\n");
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
 static int _slist1[6], _dlist1[6];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   DA = - A / tau1_nmda * q ;
   DB = - B / tau2_nmda * q ;
   DC = - C / tau1_ampa ;
   DD = - D / tau2_ampa ;
   Dw_nmda = ( gluind1 ( _threadargscomma_ glu ) * ( w_nmda_plateau - w_nmda ) - gluind2 ( _threadargscomma_ glu ) * ( w_nmda - w_nmda_0 ) ) ;
   Dw_ampa = learning_rate * active_syn ( _threadargscomma_ g_nmda ) * dopamine * ( pind_LTP ( _threadargscomma_ ca_nmdai ) * 0.5 * ( dopamine + 1.0 ) * ( w_ampa_max - w_ampa ) - pind_LTD ( _threadargscomma_ cali ) * 0.5 * ( dopamine - 1.0 ) * ( w_ampa - w_ampa_min ) ) ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 DA = DA  / (1. - dt*( ( ( - 1.0 ) / tau1_nmda )*( q ) )) ;
 DB = DB  / (1. - dt*( ( ( - 1.0 ) / tau2_nmda )*( q ) )) ;
 DC = DC  / (1. - dt*( ( - 1.0 ) / tau1_ampa )) ;
 DD = DD  / (1. - dt*( ( - 1.0 ) / tau2_ampa )) ;
 Dw_nmda = Dw_nmda  / (1. - dt*( ( ( gluind1 ( _threadargscomma_ glu ) )*( ( ( - 1.0 ) ) ) - ( gluind2 ( _threadargscomma_ glu ) )*( ( 1.0 ) ) ) )) ;
 Dw_ampa = Dw_ampa  / (1. - dt*( ( learning_rate * active_syn ( _threadargscomma_ g_nmda ) * dopamine )*( ( ( pind_LTP ( _threadargscomma_ ca_nmdai ) * 0.5 * ( dopamine + 1.0 ) )*( ( ( - 1.0 ) ) ) - ( pind_LTD ( _threadargscomma_ cali ) * 0.5 * ( dopamine - 1.0 ) )*( ( 1.0 ) ) ) ) )) ;
  return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
    A = A + (1. - exp(dt*(( ( - 1.0 ) / tau1_nmda )*( q ))))*(- ( 0.0 ) / ( ( ( - 1.0 ) / tau1_nmda )*( q ) ) - A) ;
    B = B + (1. - exp(dt*(( ( - 1.0 ) / tau2_nmda )*( q ))))*(- ( 0.0 ) / ( ( ( - 1.0 ) / tau2_nmda )*( q ) ) - B) ;
    C = C + (1. - exp(dt*(( - 1.0 ) / tau1_ampa)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau1_ampa ) - C) ;
    D = D + (1. - exp(dt*(( - 1.0 ) / tau2_ampa)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau2_ampa ) - D) ;
    w_nmda = w_nmda + (1. - exp(dt*(( ( gluind1 ( _threadargscomma_ glu ) )*( ( ( - 1.0 ) ) ) - ( gluind2 ( _threadargscomma_ glu ) )*( ( 1.0 ) ) ))))*(- ( ( ( gluind1 ( _threadargscomma_ glu ) )*( ( w_nmda_plateau ) ) - ( gluind2 ( _threadargscomma_ glu ) )*( ( ( - w_nmda_0 ) ) ) ) ) / ( ( ( gluind1 ( _threadargscomma_ glu ) )*( ( ( - 1.0 ) ) ) - ( gluind2 ( _threadargscomma_ glu ) )*( ( 1.0 ) ) ) ) - w_nmda) ;
    w_ampa = w_ampa + (1. - exp(dt*(( learning_rate * active_syn ( _threadargscomma_ g_nmda ) * dopamine )*( ( ( pind_LTP ( _threadargscomma_ ca_nmdai ) * 0.5 * ( dopamine + 1.0 ) )*( ( ( - 1.0 ) ) ) - ( pind_LTD ( _threadargscomma_ cali ) * 0.5 * ( dopamine - 1.0 ) )*( ( 1.0 ) ) ) ))))*(- ( ( ( ( learning_rate )*( active_syn ( _threadargscomma_ g_nmda ) ) )*( dopamine ) )*( ( ( ( ( pind_LTP ( _threadargscomma_ ca_nmdai ) )*( 0.5 ) )*( ( dopamine + 1.0 ) ) )*( ( w_ampa_max ) ) - ( ( ( pind_LTD ( _threadargscomma_ cali ) )*( 0.5 ) )*( ( dopamine - 1.0 ) ) )*( ( ( - w_ampa_min ) ) ) ) ) ) / ( ( ( ( learning_rate )*( active_syn ( _threadargscomma_ g_nmda ) ) )*( dopamine ) )*( ( ( ( ( pind_LTP ( _threadargscomma_ ca_nmdai ) )*( 0.5 ) )*( ( dopamine + 1.0 ) ) )*( ( ( - 1.0 ) ) ) - ( ( ( pind_LTD ( _threadargscomma_ cali ) )*( 0.5 ) )*( ( dopamine - 1.0 ) ) )*( ( 1.0 ) ) ) ) ) - w_ampa) ;
   }
  return 0;
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{    _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(Object*); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
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
 
double gluind1 (  double _lglu ) {
   double _lgluind1;
  if ( _lglu >= glu_thresh1 ) {
     _lgluind1 = 1.0 ;
     }
   else {
     _lgluind1 = 1e-6 ;
     }
    
return _lgluind1;
 }
 
static double _hoc_gluind1(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  gluind1 (  *getarg(1) );
 return(_r);
}
 
double gluind2 (  double _lglu ) {
   double _lgluind2;
  if ( _lglu <= glu_thresh2 ) {
     _lgluind2 = 1.0 ;
     }
   else {
     _lgluind2 = 0.0 ;
     }
    
return _lgluind2;
 }
 
static double _hoc_gluind2(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  gluind2 (  *getarg(1) );
 return(_r);
}
 
double pind_LTP (  double _lconc ) {
   double _lpind_LTP;
 if ( _lconc > thresh_LTP ) {
     _lpind_LTP = 1.0 ;
     }
   else {
     _lpind_LTP = 1e-6 ;
     }
   
return _lpind_LTP;
 }
 
static double _hoc_pind_LTP(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  pind_LTP (  *getarg(1) );
 return(_r);
}
 
double pind_LTD (  double _lconc ) {
   double _lpind_LTD;
 if ( _lconc > thresh_LTD ) {
     _lpind_LTD = 1.0 ;
     }
   else {
     _lpind_LTD = 1e-6 ;
     }
   
return _lpind_LTD;
 }
 
static double _hoc_pind_LTD(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  pind_LTD (  *getarg(1) );
 return(_r);
}
 
double trap (  double _lg ) {
   double _ltrap;
 if ( _lg < 1e-6 ) {
     _ltrap = 1e-6 ;
     }
   else {
     _ltrap = _lg ;
     }
   
return _ltrap;
 }
 
static double _hoc_trap(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  trap (  *getarg(1) );
 return(_r);
}
 
double active_syn (  double _lg ) {
   double _lactive_syn;
 if ( _lg < 1e-4 ) {
     _lactive_syn = 1e-6 ;
     }
   else {
     _lactive_syn = 1.0 ;
     }
   
return _lactive_syn;
 }
 
static double _hoc_active_syn(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  active_syn (  *getarg(1) );
 return(_r);
}
 
static int _ode_count(int _type){ return 6;}
 
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
	for (_i=0; _i < 6; ++_i) {
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
  w_ampa = w_ampa0;
  w_nmda = w_nmda0;
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
   w_nmda = w_nmda_0 + 2e-6 ;
   w_ampa = w_ampa_0 + 2e-6 ;
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
   g_nmda = ( B - A ) * w_nmda ;
   block = MgBlock ( _threadargs_ ) ;
   i_nmda = g_nmda * ( v - erev_nmda ) * block ;
   ica_nmda = i_nmda ;
   g_ampa = ( D - C ) * w_ampa ;
   i_ampa = g_ampa * ( v - erev_ampa ) ;
   G = g_ampa + g_nmda ;
   I = i_ampa ;
   i = I ;
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
 if(error){fprintf(stderr,"at line 153 in file adaptive_glutamate_test.mod:\n	: NMDA\n"); nrn_complain(_p); abort_run(error);}
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
 _slist1[4] = w_nmda_columnindex;  _dlist1[4] = Dw_nmda_columnindex;
 _slist1[5] = w_ampa_columnindex;  _dlist1[5] = Dw_ampa_columnindex;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "/home/shadeform/Spillover_2/mod/adaptive_glutamate_test.mod";
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
  "	POINT_PROCESS adaptive_glutamate_test\n"
  "	RANGE tau1_ampa, tau2_ampa, tau1_nmda, tau2_nmda\n"
  "	RANGE erev_ampa, erev_nmda, g, i\n"
  "	NONSPECIFIC_CURRENT i\n"
  "	\n"
  "	RANGE i_ampa, i_nmda, g_ampa, g_nmda, ratio, I, G, mg, q, alpha, eta\n"
  "	RANGE w_ampa_min, w_ampa_0, w_ampa_max, w_nmda_0, w_nmda_plateau \n"
  "	RANGE glu_thresh1, glu_thresh2, thresh_LTP, thresh_LTD, learning_rate \n"
  "	\n"
  "	POINTER glu, dopamine\n"
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
  "	erev_nmda = 15.0 (mV)\n"
  "	tau1_ampa   = 1.9       (ms)\n"
  "    tau2_ampa   = 4.8       (ms)  : tau2 > tau1\n"
  "    tau1_nmda   = 5.52      (ms)  : old value was 5.63\n"
  "    tau2_nmda   = 231       (ms)  : tau2 > tau1\n"
  "    \n"
  "    ratio       = 1         (1)   : both types give same maximal amplitude of current\n"
  "    mg          = 1         (mM)\n"
  "    alpha       = 0.062\n"
  "    q           = 2\n"
  "    eta 	= 18\n"
  "\n"
  "    w_ampa_min = 0.055 (uS)\n"
  "    w_ampa_0 = 0.11e-3 (uS)\n"
  "    w_ampa_max = 0.165e-3 (uS)\n"
  "    w_nmda_0 = 0.08e-3 (uS)\n"
  "    w_nmda_plateau = 0.8e-3 (uS)\n"
  "    glu_thresh1 = 0.05\n"
  "    glu_thresh2 = 0.05\n"
  "\n"
  "    thresh_LTP = 0.025\n"
  "    thresh_LTD = 0.005\n"
  "\n"
  "    learning_rate = 0.01\n"
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
  "	glu\n"
  "	dopamine\n"
  "        ica_nmda (nA)\n"
  "	ca_nmdai (mM)\n"
  "	cali (mM)\n"
  "}\n"
  "\n"
  "\n"
  "STATE {\n"
  "	A (uS)\n"
  "	B (uS)\n"
  "	C (uS)\n"
  "	D (uS)\n"
  "	w_nmda (uS)\n"
  "	w_ampa (uS)\n"
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
  "	w_nmda = w_nmda_0 +2e-6\n"
  "	w_ampa = w_ampa_0 +2e-6\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE state METHOD cnexp\n"
  "	\n"
  "	: NMDA\n"
  "	g_nmda = (B - A)*w_nmda\n"
  "	block  = MgBlock()\n"
  "	i_nmda = g_nmda * (v - erev_nmda) * block\n"
  "	ica_nmda = i_nmda\n"
  "	\n"
  "	: AMPA\n"
  "	g_ampa = (D - C)*w_ampa\n"
  "	i_ampa = g_ampa * (v - erev_ampa)\n"
  "	\n"
  "	: total current\n"
  "	G = g_ampa + g_nmda\n"
  "	I = i_ampa\n"
  "        i = I\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "DERIVATIVE state {\n"
  "	A' = -A/tau1_nmda*q\n"
  "	B' = -B/tau2_nmda*q\n"
  "	C' = -C/tau1_ampa\n"
  "	D' = -D/tau2_ampa\n"
  "\n"
  "	w_nmda' = (gluind1(glu)*(w_nmda_plateau - w_nmda) - gluind2(glu)*(w_nmda - w_nmda_0))\n"
  "\n"
  "        w_ampa' = learning_rate * active_syn(g_nmda)* dopamine * ( pind_LTP(ca_nmdai)*0.5*(dopamine+1)*(w_ampa_max-w_ampa) - pind_LTD(cali)*0.5*(dopamine-1)*(w_ampa - w_ampa_min) )\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "NET_RECEIVE(dummy (uS)) {\n"
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
  "FUNCTION gluind1(glu) {\n"
  "    UNITSOFF\n"
  "    if (glu >= glu_thresh1) {\n"
  "	gluind1 = 1\n"
  "    } else {\n"
  "        gluind1 = 1e-6\n"
  "    }\n"
  "    UNITSON\n"
  "}\n"
  "\n"
  "FUNCTION gluind2(glu) {\n"
  "    UNITSOFF\n"
  "    if (glu <= glu_thresh2) {\n"
  "	gluind2 = 1\n"
  "    } else {\n"
  "        gluind2 = 0\n"
  "    }\n"
  "    UNITSON\n"
  "}\n"
  "\n"
  "FUNCTION pind_LTP(conc) {\n"
  "    if (conc > thresh_LTP) {\n"
  "	pind_LTP = 1\n"
  "    } else {\n"
  "	pind_LTP = 1e-6\n"
  "    }\n"
  "}\n"
  "\n"
  "FUNCTION pind_LTD(conc) {\n"
  "    if (conc > thresh_LTD) {\n"
  "	pind_LTD = 1\n"
  "    } else {\n"
  "	pind_LTD = 1e-6\n"
  "    }\n"
  "}\n"
  "\n"
  "FUNCTION trap(g) {\n"
  "	if (g < 1e-6) {\n"
  "		trap = 1e-6\n"
  "	} else {\n"
  "	    trap = g\n"
  "        }\n"
  "}\n"
  "\n"
  "FUNCTION active_syn(g) {\n"
  "	if (g < 1e-4) {\n"
  "		active_syn = 1e-6\n"
  "	} else {\n"
  "	    active_syn = 1\n"
  "        }\n"
  "}\n"
  ;
#endif
