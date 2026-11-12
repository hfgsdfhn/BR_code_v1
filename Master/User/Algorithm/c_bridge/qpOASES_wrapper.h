/*
 * @file qpOASES_wrapper.h
 * @brief qpOASES_wrapper模块的用户接口与实现。
 */

/* 代码说明：相关模块实现。 */


/**
 *	\file interfaces/c/qpOASES_wrapper.h
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2014-2017
 *
 *	Interface that enables to call qpOASES from plain C.
 *
 */


#ifndef QPOASES_WRAPPER_H
#define QPOASES_WRAPPER_H


#ifndef QPOASES_TYPES_HPP

	/* 代码说明：相关模块实现。 */
	typedef long la_int_t;
	typedef unsigned long la_uint_t;

	/** Defines real_t for facilitating switching between double and float. */
	#ifdef __USE_SINGLE_PRECISION__
	typedef float real_t;
	#else
	typedef double real_t;
	#endif /* __USE_SINGLE_PRECISION__ */

	/** Defines int_t for facilitating switching between int and long int. */
	#ifdef __USE_LONG_INTEGERS__
	typedef long int_t;
	typedef unsigned long uint_t;
	#else
	typedef int int_t;
	typedef unsigned int uint_t;
	#endif /* __USE_LONG_INTEGERS__ */

	/* 代码说明：相关模块实现。 */
	#ifdef __USE_LONG_FINTS__
	typedef long fint_t;
	#else
	typedef int fint_t;
	#endif /* __USE_LONG_FINTS__ */

	/**
	 * Integer type for sparse matrix row/column entries. Make this "int"
	 * for 32 bit entries, and "long" for 64-bit entries on x86_64 platform.
	 *
	 * Most sparse codes still assume 32-bit entries here (HSL, BQPD, ...)
	 */
	typedef int_t sparse_int_t;

	/* 代码说明：相关模块实现。 */
	#define QProblemBClass int_t
	#define OptionsClass int_t
	#define returnValue int_t

	/* 代码说明：相关模块实现。 */
	#define HST_ZERO             0
	#define HST_IDENTITY         1
	#define HST_POSDEF           2
	#define HST_POSDEF_NULLSPACE 3
	#define HST_SEMIDEF          4
	#define HST_INDEF            5
	#define HST_UNKNOWN	         6

	/* 代码说明：相关模块实现。 */
	#define ST_LOWER            -1
	#define ST_INACTIVE          0
	#define ST_UPPER             1
	#define ST_INFEASIBLE_LOWER  2
	#define ST_INFEASIBLE_UPPER  3
	#define ST_UNDEFINED         4

	/* 代码说明：相关模块实现。 */
	#define PL_DEBUG_ITER       -2
	#define PL_TABULAR          -1
	#define PL_NONE              0
	#define PL_LOW               1
	#define PL_MEDIUM            2
	#define PL_HIGH              3

#else

	#define QProblemBClass QProblemB
	#define OptionsClass REFER_NAMESPACE_QPOASES Options

	/* 代码说明：相关模块实现。 */
	static QProblem*  globalQProblemObject  = 0;
	static QProblemB* globalQProblemBObject = 0;
	static SQProblem* globalSQProblemObject = 0;
	static Options globalOptionsObject;

#endif /* QPOASES_TYPES_HPP */



/* 代码说明：相关模块实现。 */
typedef struct
{
	int_t printLevel;						/* 代码说明：相关模块实现。 */

	int_t enableRamping;					/* 代码说明：相关模块实现。 */
	int_t enableFarBounds;					/* 代码说明：相关模块实现。 */
	int_t enableFlippingBounds;				/* 代码说明：相关模块实现。 */
	int_t enableRegularisation;				/* 代码说明：相关模块实现。 */
	int_t enableFullLITests;				/* 代码说明：相关模块实现。 */
	int_t enableNZCTests;					/* 代码说明：相关模块实现。 */
	int_t enableDriftCorrection;			/* 代码说明：相关模块实现。 */
	int_t enableCholeskyRefactorisation;	/* 代码说明：相关模块实现。 */
	int_t enableEqualities;					/* 代码说明：相关模块实现。 */

	real_t terminationTolerance;			/* 代码说明：相关模块实现。 */
	real_t boundTolerance;					/* 代码说明：相关模块实现。 */
	real_t boundRelaxation;					/* 代码说明：相关模块实现。 */
	real_t epsNum;							/* 代码说明：相关模块实现。 */
	real_t epsDen;							/* 代码说明：相关模块实现。 */
	real_t maxPrimalJump;					/* 代码说明：相关模块实现。 */
	real_t maxDualJump;						/* 代码说明：相关模块实现。 */

	real_t initialRamping;					/* 代码说明：相关模块实现。 */
	real_t finalRamping;					/* 代码说明：相关模块实现。 */
	real_t initialFarBounds;				/* 代码说明：相关模块实现。 */
	real_t growFarBounds;					/* 代码说明：相关模块实现。 */
	int_t initialStatusBounds;				/* 代码说明：相关模块实现。 */
	real_t epsFlipping;						/* 代码说明：相关模块实现。 */
	int_t numRegularisationSteps;			/* 代码说明：相关模块实现。 */
	real_t epsRegularisation;				/* 代码说明：相关模块实现。 */
	int_t numRefinementSteps;				/* 代码说明：相关模块实现。 */
	real_t epsIterRef;						/* 代码说明：相关模块实现。 */
	real_t epsLITests;						/* 代码说明：相关模块实现。 */
	real_t epsNZCTests;						/* 代码说明：相关模块实现。 */

	real_t rcondSMin;						/* 代码说明：相关模块实现。 */
	int_t enableInertiaCorrection;			/* 代码说明：相关模块实现。 */

	int_t enableDropInfeasibles;			/**< ... */
	int_t dropBoundPriority;				/**< ... */
	int_t dropEqConPriority;				/**< ... */
	int_t dropIneqConPriority;				/**< ... */
	int_t printResiduals;					/* 代码说明：相关模块实现。 */

} qpOASES_Options;


/* 功能：执行 qpOASES_Options_init 函数对应的模块操作。 */
int_t qpOASES_Options_init(	qpOASES_Options* const options, /* 成员变量。 */
							int_t mode
							);

/* 功能：执行 qpOASES_Options_copy 函数对应的模块操作。 */
int_t qpOASES_Options_copy(	const qpOASES_Options* const from, /* 成员变量。 */
							OptionsClass* const to
							);


/* 功能：执行 qpOASES_obtainOutputs 函数对应的模块操作。 */
int_t qpOASES_obtainOutputs(	const QProblemBClass* const globalQpObject, /* 成员变量。 */
								returnValue returnvalue, /* 成员变量。 */
								real_t* const x, /* 成员变量。 */
								real_t* const y, /* 成员变量。 */
								real_t* const obj, /* 成员变量。 */
								int_t* const status
								);


/* 功能：执行 QProblem_setup 函数对应的模块操作。 */
int_t QProblem_setup(	int_t nV, /* 成员变量。 */
						int_t nC, /* 成员变量。 */
						int_t hessianType
						);

/* 功能：执行 QProblem_init 函数对应的模块操作。 */
int_t QProblem_init(	const real_t* const H, /* 成员变量。 */
						const real_t* const g, /* 成员变量。 */
						const real_t* const A, /* 成员变量。 */
						const real_t* const lb, /* 成员变量。 */
						const real_t* const ub, /* 成员变量。 */
						const real_t* const lbA, /* 成员变量。 */
						const real_t* const ubA, /* 成员变量。 */
						int_t* const nWSR, /* 成员变量。 */
						real_t* const cputime, /* 成员变量。 */
						const qpOASES_Options* const options, /* 成员变量。 */
						real_t* const x, /* 成员变量。 */
						real_t* const y, /* 成员变量。 */
						real_t* const obj, /* 成员变量。 */
						int_t* const status
						);

/* 功能：执行 QProblem_hotstart 函数对应的模块操作。 */
int_t QProblem_hotstart(	const real_t* const g, /* 成员变量。 */
							const real_t* const lb, /* 成员变量。 */
							const real_t* const ub, /* 成员变量。 */
							const real_t* const lbA, /* 成员变量。 */
							const real_t* const ubA, /* 成员变量。 */
							int_t* const nWSR, /* 成员变量。 */
							real_t* const cputime, /* 成员变量。 */
							real_t* const x, /* 成员变量。 */
							real_t* const y, /* 成员变量。 */
							real_t* const obj, /* 成员变量。 */
							int_t* const status
							);

/* 功能：执行 QProblem_cleanup 函数对应的模块操作。 */
int_t QProblem_cleanup( );



/* 功能：执行 QProblemB_setup 函数对应的模块操作。 */
int_t QProblemB_setup(	int_t nV, /* 成员变量。 */
						int_t hessianType
						);

/* 功能：执行 QProblemB_init 函数对应的模块操作。 */
int_t QProblemB_init(	const real_t* const H, /* 成员变量。 */
						const real_t* const g, /* 成员变量。 */
						const real_t* const lb, /* 成员变量。 */
						const real_t* const ub, /* 成员变量。 */
						int_t* const nWSR, /* 成员变量。 */
						real_t* const cputime, /* 成员变量。 */
						const qpOASES_Options* const options, /* 成员变量。 */
						real_t* const x, /* 成员变量。 */
						real_t* const y, /* 成员变量。 */
						real_t* const obj, /* 成员变量。 */
						int_t* const status
						);

/* 功能：执行 QProblemB_hotstart 函数对应的模块操作。 */
int_t QProblemB_hotstart(	const real_t* const g, /* 成员变量。 */
							const real_t* const lb, /* 成员变量。 */
							const real_t* const ub, /* 成员变量。 */
							int_t* const nWSR, /* 成员变量。 */
							real_t* const cputime, /* 成员变量。 */
							real_t* const x, /* 成员变量。 */
							real_t* const y, /* 成员变量。 */
							real_t* const obj, /* 成员变量。 */
							int_t* const status
							);

/* 功能：执行 QProblemB_cleanup 函数对应的模块操作。 */
int_t QProblemB_cleanup( );



/* 功能：执行 SQProblem_setup 函数对应的模块操作。 */
int_t SQProblem_setup(	int_t nV, /* 成员变量。 */
						int_t nC, /* 成员变量。 */
						int_t hessianType
						);

/* 功能：执行 SQProblem_init 函数对应的模块操作。 */
int_t SQProblem_init(	const real_t* const H, /* 成员变量。 */
						const real_t* const g, /* 成员变量。 */
						const real_t* const A, /* 成员变量。 */
						const real_t* const lb, /* 成员变量。 */
						const real_t* const ub, /* 成员变量。 */
						const real_t* const lbA, /* 成员变量。 */
						const real_t* const ubA, /* 成员变量。 */
						int_t* const nWSR, /* 成员变量。 */
						real_t* const cputime, /* 成员变量。 */
						const qpOASES_Options* const options, /* 成员变量。 */
						real_t* const x, /* 成员变量。 */
						real_t* const y, /* 成员变量。 */
						real_t* const obj, /* 成员变量。 */
						int_t* const status
						);

/* 功能：执行 SQProblem_hotstart 函数对应的模块操作。 */
int_t SQProblem_hotstart(	const real_t* const H, /* 成员变量。 */
							const real_t* const g, /* 成员变量。 */
							const real_t* const A, /* 成员变量。 */
							const real_t* const lb, /* 成员变量。 */
							const real_t* const ub, /* 成员变量。 */
							const real_t* const lbA, /* 成员变量。 */
							const real_t* const ubA, /* 成员变量。 */
							int_t* const nWSR, /* 成员变量。 */
							real_t* const cputime, /* 成员变量。 */
							real_t* const x, /* 成员变量。 */
							real_t* const y, /* 成员变量。 */
							real_t* const obj, /* 成员变量。 */
							int_t* const status
							);

/* 功能：执行 SQProblem_cleanup 函数对应的模块操作。 */
int_t SQProblem_cleanup( );


#endif /* QPOASES_WRAPPER_H */


/* 代码说明：相关模块实现。 */
