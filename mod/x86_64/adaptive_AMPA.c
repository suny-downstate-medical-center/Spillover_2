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
 
#define nrn_init _nrn_init__adaptive_AMPA
#define _nrn_initial _nrn_initial__adaptive_AMPA
#define nrn_cur _nrn_cur__adaptive_AMPA
#define _nrn_current _nrn_current__adaptive_AMPA
#define nrn_jacob _nrn_jacob__adaptive_AMPA
#define nrn_state _nrn_state__adaptive_AMPA
#define _net_receive _net_receive__adaptive_AMPA 
#define reset_max reset_max__adaptive_AMPA 
#define release release__adaptive_AMPA 
 
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
#define learning_rate_w_LTP _p[5]
#define learning_rate_w_LTP_columnindex 5
#define learning_rate_w_LTD _p[6]
#define learning_rate_w_LTD_columnindex 6
#define wmax _p[7]
#define wmax_columnindex 7
#define wmin _p[8]
#define wmin_columnindex 8
#define w0 _p[9]
#define w0_columnindex 9
#define thresh_LTP _p[10]
#define thresh_LTP_columnindex 10
#define thresh_LTD _p[11]
#define thresh_LTD_columnindex 11
#define ca_nmdai_max _p[12]
#define ca_nmdai_max_columnindex 12
#define cali_max _p[13]
#define cali_max_columnindex 13
#define active_syn_flag _p[14]
#define active_syn_flag_columnindex 14
#define iAMPA _p[15]
#define iAMPA_columnindex 15
#define g _p[16]
#define g_columnindex 16
#define Rinf _p[17]
#define Rinf_columnindex 17
#define Rtau _p[18]
#define Rtau_columnindex 18
#define ina _p[19]
#define ina_columnindex 19
#define last_dopamine _p[20]
#define last_dopamine_columnindex 20
#define weight _p[21]
#define weight_columnindex 21
#define Ron _p[22]
#define Ron_columnindex 22
#define Roff _p[23]
#define Roff_columnindex 23
#define synon _p[24]
#define synon_columnindex 24
#define ca_nmdai _p[25]
#define ca_nmdai_columnindex 25
#define cali _p[26]
#define cali_columnindex 26
#define DRon _p[27]
#define DRon_columnindex 27
#define DRoff _p[28]
#define DRoff_columnindex 28
#define _g _p[29]
#define _g_columnindex 29
#define _tsav _p[30]
#define _tsav_columnindex 30
#define _nd_area  *_ppvar[0]._pval
#define _ion_ina	*_ppvar[2]._pval
#define _ion_dinadv	*_ppvar[3]._pval
#define _ion_cali	*_ppvar[4]._pval
#define _ion_ca_nmdai	*_ppvar[5]._pval
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
 static double _hoc_max(void*);
 static double _hoc_pind_LTD(void*);
 static double _hoc_pind_LTP(void*);
 static double _hoc_reset_max(void*);
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
 "max", _hoc_max,
 "pind_LTD", _hoc_pind_LTD,
 "pind_LTP", _hoc_pind_LTP,
 "reset_max", _hoc_reset_max,
 0, 0
};
#define max max_adaptive_AMPA
#define pind_LTD pind_LTD_adaptive_AMPA
#define pind_LTP pind_LTP_adaptive_AMPA
 extern double max( double , double );
 extern double pind_LTD( double );
 extern double pind_LTP( double );
 /* declare global and static user variables */
#define Cmax Cmax_adaptive_AMPA
 double Cmax = 0.1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "Cmax_adaptive_AMPA", "mM",
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
 "Cmax_adaptive_AMPA", &Cmax_adaptive_AMPA,
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
 
#define _cvode_ieq _ppvar[9]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"adaptive_AMPA",
 "Cdur",
 "Alpha",
 "Beta",
 "Erev",
 "gmax",
 "learning_rate_w_LTP",
 "learning_rate_w_LTD",
 "wmax",
 "wmin",
 "w0",
 "thresh_LTP",
 "thresh_LTD",
 "ca_nmdai_max",
 "cali_max",
 "active_syn_flag",
 0,
 "iAMPA",
 "g",
 "Rinf",
 "Rtau",
 "ina",
 "last_dopamine",
 "weight",
 0,
 "Ron",
 "Roff",
 0,
 "dopamine",
 "stimulus_flag",
 0};
 static Symbol* _na_sym;
 static Symbol* _cal_sym;
 static Symbol* _ca_nmda_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 31, _prop);
 	/*initialize range parameters*/
 	Cdur = 1.1;
 	Alpha = 1;
 	Beta = 0.5;
 	Erev = 0;
 	gmax = 1;
 	learning_rate_w_LTP = 0.01;
 	learning_rate_w_LTD = 0.01;
 	wmax = 0.006;
 	wmin = 0.001;
 	w0 = 0.00188;
 	thresh_LTP = 0.016;
 	thresh_LTD = 0.004;
 	ca_nmdai_max = 0;
 	cali_max = 0;
 	active_syn_flag = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 31;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 10, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 	_ppvar[2]._pval = &prop_ion->param[3]; /* ina */
 	_ppvar[3]._pval = &prop_ion->param[4]; /* _ion_dinadv */
 prop_ion = need_memb(_cal_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[4]._pval = &prop_ion->param[1]; /* cali */
 prop_ion = need_memb(_ca_nmda_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[5]._pval = &prop_ion->param[1]; /* ca_nmdai */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 
#define _tqitem &(_ppvar[8]._pvoid)
 static void _net_receive(Point_process*, double*, double);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _adaptive_AMPA_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("na", -10000.);
 	ion_reg("cal", 2.0);
 	ion_reg("ca_nmda", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 	_cal_sym = hoc_lookup("cal_ion");
 	_ca_nmda_sym = hoc_lookup("ca_nmda_ion");
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
  hoc_register_prop_size(_mechtype, 31, 10);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "cal_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "ca_nmda_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "pointer");
  hoc_register_dparam_semantics(_mechtype, 7, "pointer");
  hoc_register_dparam_semantics(_mechtype, 8, "netsend");
  hoc_register_dparam_semantics(_mechtype, 9, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 5;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 adaptive_AMPA /u/sid/Spillover_2/mod/adaptive_AMPA.mod\n");
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
 
double pind_LTP (  double _lconc ) {
   double _lpind_LTP;
 if ( _lconc > thresh_LTP ) {
     _lpind_LTP = 1.0 ;
     }
   else {
     _lpind_LTP = 0.0 ;
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
     _lpind_LTD = 0.0 ;
     }
   
return _lpind_LTD;
 }
 
static double _hoc_pind_LTD(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  pind_LTD (  *getarg(1) );
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
 
static int  reset_max (  ) {
   ca_nmdai_max = 0.0 ;
   cali_max = 0.0 ;
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
  cali = _ion_cali;
  ca_nmdai = _ion_ca_nmdai;
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
  cali = _ion_cali;
  ca_nmdai = _ion_ca_nmdai;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_na_sym, _ppvar, 2, 3);
   nrn_update_ion_pointer(_na_sym, _ppvar, 3, 4);
   nrn_update_ion_pointer(_cal_sym, _ppvar, 4, 1);
   nrn_update_ion_pointer(_ca_nmda_sym, _ppvar, 5, 1);
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
  cali = _ion_cali;
  ca_nmdai = _ion_ca_nmdai;
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
       weight = weight + learning_rate_w_LTP * pind_LTP ( _threadargscomma_ ca_nmdai_max ) * ( wmax - weight ) ;
       }
     else if ( last_dopamine  == - 1.0  && active_syn_flag  == 1.0 ) {
       weight = weight - learning_rate_w_LTD * pind_LTD ( _threadargscomma_ cali_max ) * ( weight - wmin ) ;
       }
     last_dopamine = dopamine ;
     reset_max ( _threadargs_ ) ;
     }
   }
 _current += ina;
 _current += iAMPA;

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
  cali = _ion_cali;
  ca_nmdai = _ion_ca_nmdai;
 _g = _nrn_current(_v + .001);
 	{ double _dina;
  _dina = ina;
 _rhs = _nrn_current(_v);
  _ion_dinadv += (_dina - ina)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g - _rhs)/.001;
  _ion_ina += ina * 1.e2/ (_nd_area);
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
  cali = _ion_cali;
  ca_nmdai = _ion_ca_nmdai;
 { error =  release();
 if(error){fprintf(stderr,"at line 125 in file adaptive_AMPA.mod:\n	SOLVE release METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 } }}

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
static const char* nmodl_filename = "/u/sid/Spillover_2/mod/adaptive_AMPA.mod";
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
  "	POINT_PROCESS adaptive_AMPA\n"
  "	RANGE R, gmax, g, ina, Alpha, Beta, iAMPA\n"
  "	USEION na WRITE ina\n"
  "	NONSPECIFIC_CURRENT  iAMPA\n"
  "	RANGE Cdur, Erev, Rinf, Rtau\n"
  "        POINTER dopamine, stimulus_flag\n"
  "	RANGE thresh_LTP, thresh_LTD, learning_rate_w_LTP, learning_rate_w_LTD, w0, wmax, wmin, w_nmda\n"
  "	RANGE ca_nmdai_max, cali_max, active_syn_flag, weight, last_dopamine\n"
  "	USEION cal READ cali VALENCE 2	\n"
  "	USEION ca_nmda READ ca_nmdai\n"
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
  "	learning_rate_w_LTP = 0.01\n"
  "    	learning_rate_w_LTD = 0.01\n"
  "    	wmax = 0.006 (uS)\n"
  "    	wmin = 0.001 (uS)\n"
  "        w0 = 0.00188 (uS)\n"
  "	\n"
  "        thresh_LTP = 0.016\n"
  "    	thresh_LTD = 0.004\n"
  "	ca_nmdai_max = 0\n"
  "	cali_max = 0\n"
  "	active_syn_flag = 0\n"
  "}\n"
  "\n"
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
  "	stimulus_flag\n"
  "        ca_nmdai        (mM)\n"
  "        cali            (mM)\n"
  "        last_dopamine\n"
  "        weight\n"
  "}\n"
  "\n"
  "STATE {Ron Roff}\n"
  "\n"
  "INITIAL {\n"
  "    Rinf = Cmax*Alpha / (Cmax*Alpha + Beta)\n"
  "    Rtau = 1 / ((Alpha * Cmax) + Beta)\n"
  "	synon = 0\n"
  "    weight = w0\n"
  "    last_dopamine = 0\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE release METHOD cnexp\n"
  "	g = (Ron + Roff)* gmax\n"
  "	iAMPA = g*(v - Erev)\n"
  "	ina = 0.9*iAMPA\n"
  "	iAMPA = 0.1*iAMPA\n"
  "\n"
  "        if (stimulus_flag == 1) {\n"
  "        	ca_nmdai_max = max(ca_nmdai, ca_nmdai_max)\n"
  "        	cali_max = max(cali, cali_max)\n"
  "		last_dopamine = dopamine\n"
  "        } else {\n"
  "	  if (last_dopamine == 1 && active_syn_flag == 1) {\n"
  "\n"
  "		  weight = weight + learning_rate_w_LTP * pind_LTP(ca_nmdai_max) * (wmax-weight)\n"
  "		  \n"
  "          } else if (last_dopamine == -1 && active_syn_flag == 1) {\n"
  "\n"
  "		  weight = weight - learning_rate_w_LTD * pind_LTD(cali_max) * (weight - wmin)\n"
  "\n"
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
  "FUNCTION pind_LTP(conc) {\n"
  "    if (conc > thresh_LTP) {\n"
  "	pind_LTP = 1\n"
  "    } else {\n"
  "	pind_LTP = 0\n"
  "    }\n"
  "}\n"
  "\n"
  "FUNCTION pind_LTD(conc) {\n"
  "    if (conc > thresh_LTD) {\n"
  "	pind_LTD = 1\n"
  "    } else {\n"
  "	pind_LTD = 0\n"
  "    }\n"
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
  "PROCEDURE reset_max() {\n"
  "	ca_nmdai_max = 0\n"
  "        cali_max = 0\n"
  "        active_syn_flag = 0\n"
  "}\n"
  ;
#endif
