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
 
#define nrn_init _nrn_init__adaptive_zahra_NMDA
#define _nrn_initial _nrn_initial__adaptive_zahra_NMDA
#define nrn_cur _nrn_cur__adaptive_zahra_NMDA
#define _nrn_current _nrn_current__adaptive_zahra_NMDA
#define nrn_jacob _nrn_jacob__adaptive_zahra_NMDA
#define nrn_state _nrn_state__adaptive_zahra_NMDA
#define _net_receive _net_receive__adaptive_zahra_NMDA 
#define release release__adaptive_zahra_NMDA 
 
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
#define mltype _p[11]
#define mltype_columnindex 11
#define mltypeMin _p[12]
#define mltypeMin_columnindex 12
#define rate_ltd _p[13]
#define rate_ltd_columnindex 13
#define rate_ltp _p[14]
#define rate_ltp_columnindex 14
#define rate_ltd_thrsh _p[15]
#define rate_ltd_thrsh_columnindex 15
#define rate_ltp_tresh _p[16]
#define rate_ltp_tresh_columnindex 16
#define treshf _p[17]
#define treshf_columnindex 17
#define tremin _p[18]
#define tremin_columnindex 18
#define iNMDA _p[19]
#define iNMDA_columnindex 19
#define g _p[20]
#define g_columnindex 20
#define synon _p[21]
#define synon_columnindex 21
#define Ron _p[22]
#define Ron_columnindex 22
#define Roff _p[23]
#define Roff_columnindex 23
#define weight _p[24]
#define weight_columnindex 24
#define tresh _p[25]
#define tresh_columnindex 25
#define Rinf _p[26]
#define Rinf_columnindex 26
#define Rtau _p[27]
#define Rtau_columnindex 27
#define B _p[28]
#define B_columnindex 28
#define ica_nmda _p[29]
#define ica_nmda_columnindex 29
#define cali _p[30]
#define cali_columnindex 30
#define ca_nmdai _p[31]
#define ca_nmdai_columnindex 31
#define DRon _p[32]
#define DRon_columnindex 32
#define DRoff _p[33]
#define DRoff_columnindex 33
#define Dweight _p[34]
#define Dweight_columnindex 34
#define Dtresh _p[35]
#define Dtresh_columnindex 35
#define _g _p[36]
#define _g_columnindex 36
#define _tsav _p[37]
#define _tsav_columnindex 37
#define _nd_area  *_ppvar[0]._pval
#define _ion_ca_nmdai	*_ppvar[2]._pval
#define _ion_ica_nmda	*_ppvar[3]._pval
#define _ion_dica_nmdadv	*_ppvar[4]._pval
#define _ion_cali	*_ppvar[5]._pval
#define dopamine	*_ppvar[6]._pval
#define _p_dopamine	_ppvar[6]._pval
 
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
 static double _hoc_funcCalsecd(void*);
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
 "funcCalsecd", _hoc_funcCalsecd,
 "funcCal", _hoc_funcCal,
 "funcCalMin", _hoc_funcCalMin,
 "mgblock", _hoc_mgblock,
 "sTresh", _hoc_sTresh,
 "supra", _hoc_supra,
 "trap", _hoc_trap,
 0, 0
};
#define _f_mgblock _f_mgblock_adaptive_zahra_NMDA
#define funcCalsecd funcCalsecd_adaptive_zahra_NMDA
#define funcCal funcCal_adaptive_zahra_NMDA
#define funcCalMin funcCalMin_adaptive_zahra_NMDA
#define mgblock mgblock_adaptive_zahra_NMDA
#define sTresh sTresh_adaptive_zahra_NMDA
#define supra supra_adaptive_zahra_NMDA
#define trap trap_adaptive_zahra_NMDA
 extern double _f_mgblock( double );
 extern double funcCalsecd( double , double );
 extern double funcCal( double , double );
 extern double funcCalMin( double );
 extern double mgblock( double );
 extern double sTresh( double , double , double );
 extern double supra( double , double , double , double , double , double );
 extern double trap( double );
 /* declare global and static user variables */
#define f f_adaptive_zahra_NMDA
 double f = 0;
#define treshltp treshltp_adaptive_zahra_NMDA
 double treshltp = 0.02;
#define usetable usetable_adaptive_zahra_NMDA
 double usetable = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "usetable_adaptive_zahra_NMDA", 0, 1,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "treshltp_adaptive_zahra_NMDA", "mM",
 "f_adaptive_zahra_NMDA", "ms",
 "Cmax", "mM",
 "Cdur", "ms",
 "Alpha", "/ms",
 "Beta", "/ms",
 "Erev", "mV",
 "mg", "mM",
 "alpha", "/mV",
 "gmax", "uS",
 "conc0", "mM",
 "mltype", "mM",
 "mltypeMin", "mM",
 "rate_ltd", "1",
 "rate_ltp", "1",
 "rate_ltd_thrsh", "1",
 "rate_ltp_tresh", "1",
 "treshf", "mM",
 "tremin", "mM",
 "iNMDA", "nA",
 "g", "uS",
 0,0
};
 static double Roff0 = 0;
 static double Ron0 = 0;
 static double delta_t = 0.01;
 static double tresh0 = 0;
 static double v = 0;
 static double weight0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "treshltp_adaptive_zahra_NMDA", &treshltp_adaptive_zahra_NMDA,
 "f_adaptive_zahra_NMDA", &f_adaptive_zahra_NMDA,
 "usetable_adaptive_zahra_NMDA", &usetable_adaptive_zahra_NMDA,
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
"adaptive_zahra_NMDA",
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
 "mltype",
 "mltypeMin",
 "rate_ltd",
 "rate_ltp",
 "rate_ltd_thrsh",
 "rate_ltp_tresh",
 "treshf",
 "tremin",
 0,
 "iNMDA",
 "g",
 "synon",
 0,
 "Ron",
 "Roff",
 "weight",
 "tresh",
 0,
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
 	_p = nrn_prop_data_alloc(_mechtype, 38, _prop);
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
 	mltype = 0;
 	mltypeMin = 0.00015;
 	rate_ltd = 0;
 	rate_ltp = 0;
 	rate_ltd_thrsh = 0;
 	rate_ltp_tresh = 0;
 	treshf = 0;
 	tremin = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 38;
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
 
#define _tqitem &(_ppvar[7]._pvoid)
 static void _net_receive(Point_process*, double*, double);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _adaptive_zahra_NMDA_reg() {
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
  hoc_register_prop_size(_mechtype, 38, 9);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "pointer");
  hoc_register_dparam_semantics(_mechtype, 7, "netsend");
  hoc_register_dparam_semantics(_mechtype, 8, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 5;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 adaptive_zahra_NMDA /www/projects/Spillover/mod/adaptive_zahra_NMDA.mod\n");
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
 
double supra (  double _ldopam , double _lwe , double _lconc , double _lcalic , double _lg_p , double _ltresh ) {
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
   _lsupra = ( ( rate_ltd * _lwe * mltype * funcCalMin ( _threadargscomma_ mltype ) * ( _ldopam - 1.0 ) ) + ( rate_ltp * funcCal ( _threadargscomma_ conc0 , _ltresh ) * ( _ldopam + 1.0 ) ) ) * trap ( _threadargscomma_ _lg_p ) * _ldopam * _ldopam ;
    
return _lsupra;
 }
 
static double _hoc_supra(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  supra (  *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) , *getarg(6) );
 return(_r);
}
 
double sTresh (  double _ldopam , double _ltresh , double _lconc0 ) {
   double _lsTresh;
  _lsTresh = ( ( rate_ltd_thrsh * ( _ltresh - tremin ) * ( _ldopam - 1.0 ) ) + rate_ltp_tresh * ( funcCalsecd ( _threadargscomma_ _lconc0 , _ltresh ) * ( _ldopam + 1.0 ) ) ) * trap ( _threadargscomma_ g ) * _ldopam * _ldopam ;
    
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
  _lfuncCal = ( 1.0 - ( 1.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 2.5 ) ) ) ) * ( 1.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 2.5 ) ) ) ;
    
return _lfuncCal;
 }
 
static double _hoc_funcCal(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  funcCal (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double funcCalsecd (  double _lcalnm , double _ltresh ) {
   double _lfuncCalsecd;
  _lfuncCalsecd = ( 1.0 - ( 1.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 2.5 ) ) ) ) * ( 1.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 2.5 ) ) ) * ( 1.0 - ( 2.0 / ( 1.0 + exp ( ( - 1000.0 * _lcalnm + ( 1000.0 * _ltresh ) ) / 2.5 ) ) ) ) ;
    
return _lfuncCalsecd;
 }
 
static double _hoc_funcCalsecd(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  funcCalsecd (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double funcCalMin (  double _lcalnm ) {
   double _lfuncCalMin;
  _lfuncCalMin = 1.0 / ( 1.0 + exp ( ( - 100000.0 * _lcalnm + ( 100000.0 * mltypeMin ) ) / 1.0 ) ) ;
    
return _lfuncCalMin;
 }
 
static double _hoc_funcCalMin(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  funcCalMin (  *getarg(1) );
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
  Roff = Roff0;
  Ron = Ron0;
  tresh = tresh0;
  weight = weight0;
 {
   Rinf = Cmax * Alpha / ( Cmax * Alpha + Beta ) ;
   Rtau = 1.0 / ( Cmax * Alpha + Beta ) ;
   synon = 0.0 ;
   weight = w0 ;
   tresh = treshf ;
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
   B = mgblock ( _threadargscomma_ v ) ;
   g = ( Ron + Roff ) * gmax * B ;
   iNMDA = g * ( v - Erev ) ;
   ica_nmda = 0.175 * iNMDA ;
   iNMDA = 0.825 * iNMDA ;
   weight = weight + supra ( _threadargscomma_ dopamine , weight , ca_nmdai , cali , g , tresh ) ;
   tresh = tresh + sTresh ( _threadargscomma_ dopamine , tresh , conc0 ) ;
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
 { error =  release();
 if(error){fprintf(stderr,"at line 76 in file adaptive_zahra_NMDA.mod:\n	SOLVE release METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
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
static const char* nmodl_filename = "/www/projects/Spillover/mod/adaptive_zahra_NMDA.mod";
static const char* nmodl_file_text = 
  "TITLE simple NMDA receptors\n"
  "NEURON {\n"
  "	POINT_PROCESS adaptive_zahra_NMDA\n"
  "	RANGE g, Alpha, Beta, Erev, gmax, Cdur, iNMDA, mg, Cmax, eta, alpha, w0, weight\n"
  "	RANGE treshf, synon, conc0, mltypeMin, mltype, rate_ltd, rate_ltp, rate_ltd_thrsh, rate_ltp_tresh, tremin\n"
  "	NONSPECIFIC_CURRENT  iNMDA\n"
  "	POINTER dopamine\n"
  "	USEION ca_nmda READ ca_nmdai WRITE ica_nmda VALENCE 2	\n"
  "	USEION cal READ cali VALENCE 2\n"
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
  "	Cdur      =1  (ms)		: transmitter duration (rising phase)\n"
  "	Alpha	= 4 (/ms /mM)	: forward (binding) rate (4)\n"
  "	Beta 	= 0.01   (/ms)   : backward (unbinding) rate\n"
  "	Erev	= 15	 (mV)		: reversal potential\n"
  "    mg   = 0      (mM)           : external magnesium concentration\n"
  "    eta = 0  :change in the code\n"
  "    alpha = 0 (/mV)\n"
  "	gmax = 0   (uS)\n"
  "    w0 = 0\n"
  "    conc0=0 (mM)\n"
  "    mltype=0 (mM)\n"
  "    mltypeMin=0.00015 (mM)\n"
  "    rate_ltd=0 (1)\n"
  "    rate_ltp=0 (1)\n"
  "    rate_ltd_thrsh=0 (1)\n"
  "    rate_ltp_tresh=0 (1)\n"
  "    treshf=0 (mM)    :change in the code\n"
  "    treshltp=0.02(mM)\n"
  "    tremin=0 (mM)\n"
  "    f=0   (ms)\n"
  "    \n"
  "\n"
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
  "    B                       : magnesium block\n"
  "	ica_nmda        (nA)\n"
  "	dopamine\n"
  "    cali            (mM)\n"
  "	ca_nmdai        (mM)\n"
  "	\n"
  "}\n"
  "\n"
  "STATE {Ron Roff weight tresh}\n"
  "\n"
  "INITIAL {\n"
  "	Rinf = Cmax*Alpha / (Cmax*Alpha + Beta)\n"
  "	Rtau = 1 / (Cmax*Alpha + Beta)\n"
  "	synon = 0\n"
  "	weight = w0\n"
  "    tresh = treshf\n"
  "   \n"
  "\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE release METHOD cnexp\n"
  "    B = mgblock(v)\n"
  "	g = (Ron + Roff)* gmax*B\n"
  "	iNMDA = g*(v - Erev)\n"
  "    ica_nmda = 0.175*iNMDA   :(5-10 times more permeable to Ca++ than Na+ or K+, Ascher and Nowak, 1988)\n"
  "    iNMDA = 0.825*iNMDA\n"
  "    weight=weight+supra(dopamine,weight,ca_nmdai,cali,g,tresh)\n"
  "    tresh=tresh+sTresh(dopamine,tresh,conc0)\n"
  "\n"
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
  "FUNCTION supra(dopam,we(uS),conc(mM),calic(mM),g_p(uS),tresh(mM))(uS/ms) {\n"
  "    UNITSOFF\n"
  "\n"
  "  \n"
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
  "	supra =((rate_ltd*we*mltype*funcCalMin(mltype)*(dopam-1))+(rate_ltp*funcCal(conc0,tresh)*(dopam+1)))*trap(g_p)*dopam*dopam\n"
  "	\n"
  "	UNITSON    \n"
  "}\n"
  "FUNCTION sTresh(dopam,tresh(mM),conc0)(uM/ms) {\n"
  "    UNITSOFF\n"
  "    sTresh =((rate_ltd_thrsh*(tresh-tremin)*(dopam-1))+rate_ltp_tresh*(funcCalsecd(conc0,tresh)*(dopam+1)))*trap(g)*dopam*dopam\n"
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
  "    funcCal= (1-(1 / (1 + exp((-1000*calnm + (1000*tresh))/2.5))))*(1 / (1 + exp((-1000*calnm+(1000*tresh))/2.5))) \n"
  "    UNITSON\n"
  "}\n"
  "FUNCTION funcCalsecd(calnm(mM),tresh(mM))() {\n"
  "    UNITSOFF\n"
  "    funcCalsecd= (1-(1 / (1 +exp((-1000*calnm+ (1000*tresh))/2.5))))*(1 / (1 + exp((-1000*calnm+(1000*tresh))/2.5)))* (1-(2 / (1 + exp((-1000*calnm+ (1000*tresh))/2.5))))\n"
  "    UNITSON\n"
  "}\n"
  "FUNCTION funcCalMin(calnm(mM))() {\n"
  "    UNITSOFF\n"
  "    funcCalMin= 1 / (1 + exp((-100000*calnm+(100000*mltypeMin))/1)) \n"
  "    UNITSON\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  ;
#endif
