/*
 * @file qpOASES_wrapper.h
 * @brief qpOASES_wrapper模块的用户接口与实现。
 */

#ifndef QPOASES_WRAPPER_H
#define QPOASES_WRAPPER_H

#ifndef QPOASES_TYPES_HPP

	typedef long la_int_t;
	typedef unsigned long la_uint_t;

	#ifdef __USE_SINGLE_PRECISION__
	typedef float real_t;
	#else
	typedef double real_t;
	#endif

	#ifdef __USE_LONG_INTEGERS__
	typedef long int_t;
	typedef unsigned long uint_t;
	#else
	typedef int int_t;
	typedef unsigned int uint_t;
	#endif

	#ifdef __USE_LONG_FINTS__
	typedef long fint_t;
	#else
	typedef int fint_t;
	#endif

	typedef int_t sparse_int_t;

	#define QProblemBClass int_t
	#define OptionsClass int_t
	#define returnValue int_t

	#define HST_ZERO             0
	#define HST_IDENTITY         1
	#define HST_POSDEF           2
	#define HST_POSDEF_NULLSPACE 3
	#define HST_SEMIDEF          4
	#define HST_INDEF            5
	#define HST_UNKNOWN	         6

	#define ST_LOWER            -1
	#define ST_INACTIVE          0
	#define ST_UPPER             1
	#define ST_INFEASIBLE_LOWER  2
	#define ST_INFEASIBLE_UPPER  3
	#define ST_UNDEFINED         4

	#define PL_DEBUG_ITER       -2
	#define PL_TABULAR          -1
	#define PL_NONE              0
	#define PL_LOW               1
	#define PL_MEDIUM            2
	#define PL_HIGH              3

#else

	#define QProblemBClass QProblemB
	#define OptionsClass REFER_NAMESPACE_QPOASES Options

	static QProblem*  globalQProblemObject  = 0;
	static QProblemB* globalQProblemBObject = 0;
	static SQProblem* globalSQProblemObject = 0;
	static Options globalOptionsObject;

#endif

typedef struct
{
	int_t printLevel;

	int_t enableRamping;
	int_t enableFarBounds;
	int_t enableFlippingBounds;
	int_t enableRegularisation;
	int_t enableFullLITests;
	int_t enableNZCTests;
	int_t enableDriftCorrection;
	int_t enableCholeskyRefactorisation;
	int_t enableEqualities;

	real_t terminationTolerance;
	real_t boundTolerance;
	real_t boundRelaxation;
	real_t epsNum;
	real_t epsDen;
	real_t maxPrimalJump;
	real_t maxDualJump;

	real_t initialRamping;
	real_t finalRamping;
	real_t initialFarBounds;
	real_t growFarBounds;
	int_t initialStatusBounds;
	real_t epsFlipping;
	int_t numRegularisationSteps;
	real_t epsRegularisation;
	int_t numRefinementSteps;
	real_t epsIterRef;
	real_t epsLITests;
	real_t epsNZCTests;

	real_t rcondSMin;
	int_t enableInertiaCorrection;

	int_t enableDropInfeasibles;			/**< ... */
	int_t dropBoundPriority;				/**< ... */
	int_t dropEqConPriority;				/**< ... */
	int_t dropIneqConPriority;				/**< ... */
	int_t printResiduals;

} qpOASES_Options;

/* 功能：初始化qpOASES_Options_init对应的外设、参数和运行状态。 */
int_t qpOASES_Options_init(	qpOASES_Options* const options,
							int_t mode
							);

/* 功能：处理qpOASES_Options_copy对应的输入并更新相关状态。 */
int_t qpOASES_Options_copy(	const qpOASES_Options* const from,
							OptionsClass* const to
							);

/* 功能：处理qpOASES_obtainOutputs对应的输入并更新相关状态。 */
int_t qpOASES_obtainOutputs(	const QProblemBClass* const globalQpObject,
								returnValue returnvalue,
								real_t* const x,
								real_t* const y,
								real_t* const obj,
								int_t* const status
								);

/* 功能：处理QProblem_setup对应的输入并更新相关状态。 */
int_t QProblem_setup(	int_t nV,
						int_t nC,
						int_t hessianType
						);

/* 功能：初始化QProblem_init对应的外设、参数和运行状态。 */
int_t QProblem_init(	const real_t* const H,
						const real_t* const g,
						const real_t* const A,
						const real_t* const lb,
						const real_t* const ub,
						const real_t* const lbA,
						const real_t* const ubA,
						int_t* const nWSR,
						real_t* const cputime,
						const qpOASES_Options* const options,
						real_t* const x,
						real_t* const y,
						real_t* const obj,
						int_t* const status
						);

/* 功能：处理QProblem_hotstart对应的输入并更新相关状态。 */
int_t QProblem_hotstart(	const real_t* const g,
							const real_t* const lb,
							const real_t* const ub,
							const real_t* const lbA,
							const real_t* const ubA,
							int_t* const nWSR,
							real_t* const cputime,
							real_t* const x,
							real_t* const y,
							real_t* const obj,
							int_t* const status
							);

/* 功能：重置或释放QProblem_cleanup对应的内部状态和资源。 */
int_t QProblem_cleanup( );

/* 功能：处理QProblemB_setup对应的输入并更新相关状态。 */
int_t QProblemB_setup(	int_t nV,
						int_t hessianType
						);

/* 功能：初始化QProblemB_init对应的外设、参数和运行状态。 */
int_t QProblemB_init(	const real_t* const H,
						const real_t* const g,
						const real_t* const lb,
						const real_t* const ub,
						int_t* const nWSR,
						real_t* const cputime,
						const qpOASES_Options* const options,
						real_t* const x,
						real_t* const y,
						real_t* const obj,
						int_t* const status
						);

/* 功能：处理QProblemB_hotstart对应的输入并更新相关状态。 */
int_t QProblemB_hotstart(	const real_t* const g,
							const real_t* const lb,
							const real_t* const ub,
							int_t* const nWSR,
							real_t* const cputime,
							real_t* const x,
							real_t* const y,
							real_t* const obj,
							int_t* const status
							);

/* 功能：重置或释放QProblemB_cleanup对应的内部状态和资源。 */
int_t QProblemB_cleanup( );

/* 功能：处理SQProblem_setup对应的输入并更新相关状态。 */
int_t SQProblem_setup(	int_t nV,
						int_t nC,
						int_t hessianType
						);

/* 功能：初始化SQProblem_init对应的外设、参数和运行状态。 */
int_t SQProblem_init(	const real_t* const H,
						const real_t* const g,
						const real_t* const A,
						const real_t* const lb,
						const real_t* const ub,
						const real_t* const lbA,
						const real_t* const ubA,
						int_t* const nWSR,
						real_t* const cputime,
						const qpOASES_Options* const options,
						real_t* const x,
						real_t* const y,
						real_t* const obj,
						int_t* const status
						);

/* 功能：处理SQProblem_hotstart对应的输入并更新相关状态。 */
int_t SQProblem_hotstart(	const real_t* const H,
							const real_t* const g,
							const real_t* const A,
							const real_t* const lb,
							const real_t* const ub,
							const real_t* const lbA,
							const real_t* const ubA,
							int_t* const nWSR,
							real_t* const cputime,
							real_t* const x,
							real_t* const y,
							real_t* const obj,
							int_t* const status
							);

/* 功能：重置或释放SQProblem_cleanup对应的内部状态和资源。 */
int_t SQProblem_cleanup( );

#endif
