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
 
#define nrn_init _nrn_init__adaptive_cshom_AMPA
#define _nrn_initial _nrn_initial__adaptive_cshom_AMPA
#define nrn_cur _nrn_cur__adaptive_cshom_AMPA
#define _nrn_current _nrn_current__adaptive_cshom_AMPA
#define nrn_jacob _nrn_jacob__adaptive_cshom_AMPA
#define nrn_state _nrn_state__adaptive_cshom_AMPA
#define _net_receive _net_receive__adaptive_cshom_AMPA 
#define reset_max reset_max__adaptive_cshom_AMPA 
#define release release__adaptive_cshom_AMPA 
 
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
#define Cdur _p[0]
#define Cdur_columnindex 0
#define Alpha _p[1]
#define Alpha_columnindex 1
#define Beta _p[2]
#define Beta_columnindex 2
#define Erev _p[3]
#define Erev_columnindex 3
#define gmax _p[4]
#define gmax_columnindex 4
#define learning_rate _p[5]
#define learning_rate_columnindex 5
#define learning_rate_w_LTP _p[6]
#define learning_rate_w_LTP_columnindex 6
#define learning_rate_w_LTD _p[7]
#define learning_rate_w_LTD_columnindex 7
#define wmax _p[8]
#define wmax_columnindex 8
#define wmin _p[9]
#define wmin_columnindex 9
#define w0 _p[10]
#define w0_columnindex 10
#define ca_nmdai_max _p[11]
#define ca_nmdai_max_columnindex 11
#define cali_max _p[12]
#define cali_max_columnindex 12
#define active_syn_flag _p[13]
#define active_syn_flag_columnindex 13
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
#define LTD_thresh_factor _p[25]
#define LTD_thresh_factor_columnindex 25
#define steepness_LTP _p[26]
#define steepness_LTP_columnindex 26
#define steepness_LTD _p[27]
#define steepness_LTD_columnindex 27
#define iAMPA _p[28]
#define iAMPA_columnindex 28
#define g _p[29]
#define g_columnindex 29
#define Rinf _p[30]
#define Rinf_columnindex 30
#define Rtau _p[31]
#define Rtau_columnindex 31
#define ina _p[32]
#define ina_columnindex 32
#define last_dopamine _p[33]
#define last_dopamine_columnindex 33
#define weight _p[34]
#define weight_columnindex 34
#define lthresh_LTP _p[35]
#define lthresh_LTP_columnindex 35
#define lthresh_LTD _p[36]
#define lthresh_LTD_columnindex 36
#define hthresh_LTP _p[37]
#define hthresh_LTP_columnindex 37
#define delta_LTP _p[38]
#define delta_LTP_columnindex 38
#define Ron _p[39]
#define Ron_columnindex 39
#define Roff _p[40]
#define Roff_columnindex 40
#define synon _p[41]
#define synon_columnindex 41
#define ca_nmdai _p[42]
#define ca_nmdai_columnindex 42
#define cali _p[43]
#define cali_columnindex 43
#define deriv _p[44]
#define deriv_columnindex 44
#define ica_nmda _p[45]
#define ica_nmda_columnindex 45
#define DRon _p[46]
#define DRon_columnindex 46
#define DRoff _p[47]
#define DRoff_columnindex 47
#define _g _p[48]
#define _g_columnindex 48
#define _tsav _p[49]
#define _tsav_columnindex 49
#define _nd_area  *_ppvar[0]._pval
#define _ion_ina	*_ppvar[2]._pval
#define _ion_dinadv	*_ppvar[3]._pval
#define _ion_ca_nmdai	*_ppvar[4]._pval
#define _ion_ica_nmda	*_ppvar[5]._pval
#define _ion_dica_nmdadv	*_ppvar[6]._pval
#define _ion_cali	*_ppvar[7]._pval
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
 "hthresh", _hoc_hthresh,
 "lthresh", _hoc_lthresh,
 "min", _hoc_min,
 "max", _hoc_max,
 "reset_max", _hoc_reset_max,
 "sigmoidal", _hoc_sigmoidal,
 0, 0
};
#define hthresh hthresh_adaptive_cshom_AMPA
#define lthresh lthresh_adaptive_cshom_AMPA
#define min min_adaptive_cshom_AMPA
#define max max_adaptive_cshom_AMPA
#define sigmoidal sigmoidal_adaptive_cshom_AMPA
 extern double hthresh( double , double , double );
 extern double lthresh( double , double , double );
 extern double min( double , double );
 extern double max( double , double );
 extern double sigmoidal( double , double , double );
 /* declare global and static user variables */
#define Cmax Cmax_adaptive_cshom_AMPA
 double Cmax = 0.1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "Cmax_adaptive_cshom_AMPA", "mM",
 "Cdur", "ms",
 "Alpha", "/ms",
 "Beta", "/ms",
 "Erev", "mV",
 "gmax", "uS",
 "wmax", "uS",
 "wmin", "uS",
 "w0", "uS",
 "iAMPA", "nA",
 "g", "uS",
 "Rtau", "ms",
 0,0
};
 static double Roff0 = 0;
 static double Ron0 = 0;
 static double delta_t = 0.01;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "Cmax_adaptive_cshom_AMPA", &Cmax_adaptive_cshom_AMPA,
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
"adaptive_cshom_AMPA",
 "Cdur",
 "Alpha",
 "Beta",
 "Erev",
 "gmax",
 "learning_rate",
 "learning_rate_w_LTP",
 "learning_rate_w_LTD",
 "wmax",
 "wmin",
 "w0",
 "ca_nmdai_max",
 "cali_max",
 "active_syn_flag",
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
 "LTD_thresh_factor",
 "steepness_LTP",
 "steepness_LTD",
 0,
 "iAMPA",
 "g",
 "Rinf",
 "Rtau",
 "ina",
 "last_dopamine",
 "weight",
 "lthresh_LTP",
 "lthresh_LTD",
 "hthresh_LTP",
 "delta_LTP",
 0,
 "Ron",
 "Roff",
 0,
 "dopamine",
 "stimulus_flag",
 0};
 static Symbol* _na_sym;
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
 	_p = nrn_prop_data_alloc(_mechtype, 50, _prop);
 	/*initialize range parameters*/
 	Cdur = 1.1;
 	Alpha = 1;
 	Beta = 0.5;
 	Erev = 0;
 	gmax = 1;
 	learning_rate = 0.01;
 	learning_rate_w_LTP = 0.01;
 	learning_rate_w_LTD = 0.01;
 	wmax = 0.006;
 	wmin = 0.001;
 	w0 = 0.00188;
 	ca_nmdai_max = 0;
 	cali_max = 0;
 	active_syn_flag = 1e-06;
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
 	LTD_thresh_factor = 1;
 	steepness_LTP = 0.25;
 	steepness_LTD = 2.5;
  }
 	_prop->param = _p;
 	_prop->param_size = 50;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 12, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 	_ppvar[2]._pval = &prop_ion->param[3]; /* ina */
 	_ppvar[3]._pval = &prop_ion->param[4]; /* _ion_dinadv */
 prop_ion = need_memb(_ca_nmda_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[4]._pval = &prop_ion->param[1]; /* ca_nmdai */
 	_ppvar[5]._pval = &prop_ion->param[3]; /* ica_nmda */
 	_ppvar[6]._pval = &prop_ion->param[4]; /* _ion_dica_nmdadv */
 prop_ion = need_memb(_cal_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[7]._pval = &prop_ion->param[1]; /* cali */
 
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

 void _adaptive_cshom_AMPA_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("na", -10000.);
 	ion_reg("ca_nmda", 2.0);
 	ion_reg("cal", 2.0);
 	_na_sym = hoc_lookup("na_ion");
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
  hoc_register_prop_size(_mechtype, 50, 12);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 7, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 8, "pointer");
  hoc_register_dparam_semantics(_mechtype, 9, "pointer");
  hoc_register_dparam_semantics(_mechtype, 10, "netsend");
  hoc_register_dparam_semantics(_mechtype, 11, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 5;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 adaptive_cshom_AMPA /www/projects/Spillover/mod/adaptive_cshom_AMPA.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "simple AMPA receptors";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int reset_max();
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
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
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{    _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = 0;}
 {
   if ( _lflag  == 0.0 ) {
     active_syn_flag = 1.0 ;
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
   active_syn_flag = 1e-6 ;
    return 0; }
 
static double _hoc_reset_max(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r = 1.;
 reset_max (  );
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
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_na_sym, _ppvar, 2, 3);
   nrn_update_ion_pointer(_na_sym, _ppvar, 3, 4);
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 4, 1);
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 5, 3);
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 6, 4);
   nrn_update_ion_pointer(_cal_sym, _ppvar, 7, 1);
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
   Rtau = 1.0 / ( ( Alpha * Cmax ) + Beta ) ;
   synon = 0.0 ;
   weight = w0 ;
   lthresh_LTP = thresh_LTP_0 ;
   lthresh_LTD = thresh_LTD_0 ;
   hthresh_LTP = hthresh_LTP_0 ;
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
   g = ( Ron + Roff ) * gmax ;
   iAMPA = g * ( v - Erev ) ;
   ina = 0.9 * iAMPA ;
   iAMPA = 0.1 * iAMPA ;
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
       weight = weight - learning_rate_w_LTD * sigmoidal ( _threadargscomma_ cali_max , lthresh_LTD , steepness_LTD ) * weight ;
       lthresh_LTP = lthresh_LTP - learning_rate_thresh_LTP * ( lthresh_LTP - lthresh_LTP_min ) ;
       }
     last_dopamine = dopamine ;
     reset_max ( _threadargs_ ) ;
     }
   }
 _current += ina;
 _current += iAMPA;
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
 double _dina;
  _dina = ina;
  _dica_nmda = ica_nmda;
 _rhs = _nrn_current(_v);
  _ion_dinadv += (_dina - ina)/.001 * 1.e2/ (_nd_area);
  _ion_dica_nmdadv += (_dica_nmda - ica_nmda)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g - _rhs)/.001;
  _ion_ina += ina * 1.e2/ (_nd_area);
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
 { error =  release();
 if(error){fprintf(stderr,"at line 150 in file adaptive_cshom_AMPA.mod:\n        SOLVE release METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 }  }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = Ron_columnindex;  _dlist1[0] = DRon_columnindex;
 _slist1[1] = Roff_columnindex;  _dlist1[1] = DRoff_columnindex;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "/www/projects/Spillover/mod/adaptive_cshom_AMPA.mod";
static const char* nmodl_file_text = 
  "TITLE simple AMPA receptors\n"
  "\n"
  "COMMENT\n"
  "-----------------------------------------------------------------------------\n"
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
  "Modified by Penny under the instruction of M.L.Hines on Oct 03, 2017\n"
  "	Change gmax\n"
  "\n"
  "-----------------------------------------------------------------------------\n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS adaptive_cshom_AMPA\n"
  "	RANGE R, gmax, g, ina, Alpha, Beta, iAMPA\n"
  "	USEION na WRITE ina\n"
  "	NONSPECIFIC_CURRENT  iAMPA\n"
  "	RANGE Cdur, Erev, Rinf, Rtau\n"
  "        RANGE weight, lthresh_LTP, lthresh_LTD, hthresh_LTP, last_dopamine, lthresh_LTP_min\n"
  "        POINTER dopamine, stimulus_flag\n"
  "	RANGE thresh_LTP, thresh_LTD, learning_rate, w0, wmax, wmin, steepness_LTP, steepness_LTD\n"
  "	RANGE learning_rate_w_LTP, learning_rate_w_LTD, thresh_LTP_0, learning_rate_thresh_LTP, thresh_LTD_0, learning_rate_thresh_LTD \n"
  "	RANGE hthresh_LTP_const, hthresh_LTP_0, hthresh_max, n, delta, LTD_thresh_factor, width, delta_LTP\n"
  "	RANGE ca_nmdai_max, cali_max, active_syn_flag, Cdur_init, Cdur_factor\n"
  "	USEION ca_nmda READ ca_nmdai WRITE ica_nmda VALENCE 2	\n"
  "	USEION cal READ cali VALENCE 2\n"
  "}\n"
  "UNITS {\n"
  "	(nA) = (nanoamp)\n"
  "	(mV) = (millivolt)\n"
  "	(uS) = (microsiemens)\n"
  "	(mM) = (milli/liter)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    Cmax	= 0.1	(mM)		: max transmitter concentration\n"
  ":	Cdur	= 0.3	(ms)		: transmitter duration (rising phase)\n"
  "	Cdur	= 1.1	(ms)		: transmitter duration (rising phase)\n"
  ":	Alpha	= 0.94	(/ms)	: forward (binding) rate\n"
  "	Alpha	= 1	(/ms)	: forward (binding) rate\n"
  ":	Beta	= 0.018	(/ms)		: backward (unbinding) rate\n"
  "	Beta	= 0.5 (/ms)		: backward (unbinding) rate\n"
  "	Erev	= 0	(mV)		:0 reversal potential\n"
  "	gmax    = 1  (uS)\n"
  "    	\n"
  "	learning_rate = 0.01\n"
  "	learning_rate_w_LTP = 0.01\n"
  "	learning_rate_w_LTD = 0.01\n"
  "    	wmax = 0.006 (uS)\n"
  "    	wmin = 0.001 (uS)\n"
  "        w0 = 0.00188 (uS)\n"
  "	\n"
  "	ca_nmdai_max = 0\n"
  "	cali_max = 0\n"
  "	active_syn_flag = 1e-6\n"
  "\n"
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
  "        LTD_thresh_factor = 1.0\n"
  "	steepness_LTP = 0.25\n"
  "	steepness_LTD = 2.5\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v		(mV)		: postsynaptic voltage\n"
  "	iAMPA 		(nA)		: current = g*(v - Erev)\n"
  "	g 		(uS)		: conductance\n"
  "	Rinf				: steady state channels open\n"
  "	Rtau		(ms)		: time constant of channel binding\n"
  "	synon\n"
  "	ina\n"
  "        dopamine\n"
  "	last_dopamine\n"
  "	stimulus_flag\n"
  "        ca_nmdai        (mM)\n"
  "        cali            (mM)\n"
  "        deriv\n"
  "        ica_nmda        (nA)\n"
  "	weight\n"
  "	lthresh_LTP\n"
  "	lthresh_LTD\n"
  "	hthresh_LTP\n"
  "	delta_LTP\n"
  "}\n"
  "\n"
  "STATE {Ron Roff}\n"
  "\n"
  "INITIAL {\n"
  "    Rinf = Cmax*Alpha / (Cmax*Alpha + Beta)\n"
  "    Rtau = 1 / ((Alpha * Cmax) + Beta)\n"
  "    synon = 0\n"
  "    weight = w0\n"
  "    lthresh_LTP = thresh_LTP_0\n"
  "    lthresh_LTD = thresh_LTD_0\n"
  "    hthresh_LTP = hthresh_LTP_0\n"
  "    last_dopamine = 0\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "        SOLVE release METHOD cnexp\n"
  "	g = (Ron + Roff)* gmax\n"
  "	iAMPA = g*(v - Erev)\n"
  "	ina = 0.9*iAMPA\n"
  "	iAMPA = 0.1*iAMPA\n"
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
  "		  weight = weight - learning_rate_w_LTD * sigmoidal(cali_max, lthresh_LTD, steepness_LTD) * weight \n"
  "		  lthresh_LTP = lthresh_LTP - learning_rate_thresh_LTP * (lthresh_LTP - lthresh_LTP_min)\n"
  "\n"
  "          }\n"
  "          last_dopamine = dopamine		\n"
  "          reset_max()\n"
  "}\n"
  "\n"
  "}\n"
  "\n"
  "DERIVATIVE release {\n"
  "	Ron' = (synon*Rinf - Ron)/Rtau\n"
  "	Roff' = -Beta*Roff\n"
  "}\n"
  "\n"
  ": following supports both saturation from single input and\n"
  ": summation from multiple inputs\n"
  ": if spike occurs during CDur then new off time is t + CDur\n"
  ": ie. transmitter concatenates but does not summate\n"
  ": Note: automatic initialization of all reference args to 0 except first\n"
  "\n"
  "NET_RECEIVE(dummy, on, nspike, r0, t0 (ms)) {\n"
  "	: flag is an implicit argument of NET_RECEIVE and  normally 0\n"
  "        if (flag == 0) { : a spike, so turn on if not already in a Cdur pulse\n"
  "		active_syn_flag = 1\n"
  "		nspike = nspike + 1\n"
  "		if (!on) {\n"
  "			r0 = r0*exp(-Beta*(t - t0))\n"
  "			t0 = t\n"
  "			on = 1\n"
  "			synon = synon + weight\n"
  "			state_discontinuity(Ron, Ron + r0)\n"
  "			state_discontinuity(Roff, Roff - r0)\n"
  "		}\n"
  "		: come again in Cdur with flag = current value of nspike\n"
  "		net_send(Cdur, nspike)\n"
  "        }\n"
  "	if (flag == nspike) { : if this associated with last spike then turn off\n"
  "		r0 = weight*Rinf + (r0 - weight*Rinf)*exp(-(t - t0)/Rtau)\n"
  "		t0 = t\n"
  "		synon = synon - weight\n"
  "		state_discontinuity(Ron, Ron - r0)\n"
  "		state_discontinuity(Roff, Roff + r0)\n"
  "		on = 0\n"
  "	}\n"
  "\n"
  "}\n"
  "\n"
  "FUNCTION lthresh(conc, KD, steepness) {\n"
  ":    lthresh = conc^n/(KD^n + conc^n) \n"
  "    lthresh = sigmoidal(conc,KD, steepness) \n"
  "}\n"
  "\n"
  "FUNCTION hthresh(conc, KD, steepness) {\n"
  ":    hthresh = KD^n/(KD^n + conc^n)\n"
  "    hthresh = 1 - sigmoidal(conc, KD, steepness) \n"
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
  "PROCEDURE reset_max() {\n"
  "	ca_nmdai_max = 0\n"
  "        cali_max = 0\n"
  "        active_syn_flag = 1e-6\n"
  "}\n"
  "\n"
  "FUNCTION sigmoidal(x, x_offset, s) {\n"
  "    sigmoidal = 1/(1+exp(- s *(x - x_offset)))\n"
  "}\n"
  ;
#endif
