/*
 * @file qpOASES_wrapper.cpp
 * @brief qpOASES_wrapper模块的用户接口与实现。
 */

/* 代码说明：相关模块实现。 */


/**
 *	\file interfaces/c/qpOASES_wrapper.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2014-2017
 *
 *	Interface that enables to call qpOASES from plain C.
 *
 */


#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES


extern "C" {
#include "qpOASES_wrapper.h"
}




/*
 *	q p O A S E S _ O p t i o n s _ i n i t
 */
int_t qpOASES_Options_init(	qpOASES_Options* const options,
							int_t mode
							)
{
	if ( ( mode < 0 ) || ( mode > 2 ) )
		return -1;

	if ( options == 0 )
		return -1;


	/* 代码说明：相关模块实现。 */
	options->printLevel = PL_MEDIUM;
	#ifdef __DEBUG__
	options->printLevel = PL_HIGH;
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	options->printLevel = PL_NONE;
	#endif

	options->enableRamping                 =  BT_TRUE;
	options->enableFarBounds               =  BT_TRUE;
	options->enableFlippingBounds          =  BT_TRUE;
	options->enableRegularisation          =  BT_FALSE;
	options->enableFullLITests             =  BT_FALSE;
	options->enableNZCTests                =  BT_TRUE;
	options->enableDriftCorrection         =  1;
	options->enableCholeskyRefactorisation =  0;
	options->enableEqualities              =  BT_FALSE;

	#ifdef __USE_SINGLE_PRECISION__
	options->terminationTolerance          =  1.0e2 * EPS;
	options->boundTolerance                =  1.0e2 * EPS;
	#else
	options->terminationTolerance          =  5.0e6 * EPS;
	options->boundTolerance                =  1.0e6 * EPS;
	#endif
	options->boundRelaxation               =  1.0e4;
	#ifdef __USE_SINGLE_PRECISION__
	options->epsNum                        = -1.0e2 * EPS;
	options->epsDen                        =  1.0e2 * EPS;
	#else
	options->epsNum                        = -1.0e3 * EPS;
	options->epsDen                        =  1.0e3 * EPS;
	#endif
	options->maxPrimalJump                 =  1.0e8;
	options->maxDualJump                   =  1.0e8;

	options->initialRamping                =  0.5;
	options->finalRamping                  =  1.0;
	options->initialFarBounds              =  1.0e6;
	options->growFarBounds                 =  1.0e3;
 	options->initialStatusBounds           =  ST_LOWER;
	#ifdef __USE_SINGLE_PRECISION__
	options->epsFlipping                   =  5.0e1 * EPS;
	#else
	options->epsFlipping                   =  1.0e3 * EPS;
	#endif
	options->numRegularisationSteps        =  0;
	#ifdef __USE_SINGLE_PRECISION__
	options->epsRegularisation             =  2.0e1 * EPS;
	options->numRefinementSteps            =  2;
	#else
	options->epsRegularisation             =  1.0e3 * EPS;
	options->numRefinementSteps            =  1;
	#endif
	options->epsIterRef                    =  1.0e2 * EPS;
	#ifdef __USE_SINGLE_PRECISION__
	options->epsLITests                    =  5.0e1 * EPS;
	options->epsNZCTests                   =  1.0e2 * EPS;
	#else
	options->epsLITests                    =  1.0e5 * EPS;
	options->epsNZCTests                   =  3.0e3 * EPS;
	#endif

	options->enableDropInfeasibles         =  BT_FALSE;
    options->dropBoundPriority             =  1;
    options->dropEqConPriority             =  1;
    options->dropIneqConPriority           =  1;


	switch ( mode )
	{
		case 0:
			/* 代码说明：相关模块实现。 */
			break;


		case 1:
			/* 代码说明：相关模块实现。 */
			options->enableFullLITests             =  BT_TRUE;
			options->enableCholeskyRefactorisation =  1;

			#ifdef __USE_SINGLE_PRECISION__
			options->numRefinementSteps            =  3;
			#else
			options->numRefinementSteps            =  2;
			#endif


		case 2:
			/* 代码说明：相关模块实现。 */
			options->enableRamping                 =  BT_FALSE;
			options->enableFarBounds               =  BT_TRUE;
			options->enableFlippingBounds          =  BT_FALSE;
			options->enableRegularisation          =  BT_TRUE;
			options->enableNZCTests                =  BT_FALSE;
			options->enableDriftCorrection         =  0;
			options->enableEqualities              =  BT_TRUE;

			#ifdef __USE_SINGLE_PRECISION__
			options->terminationTolerance          =  1.0e3 * EPS;
			#else
			options->terminationTolerance          =  1.0e9 * EPS;
			#endif

			options->initialStatusBounds           =  ST_INACTIVE;
			options->numRegularisationSteps        =  1;
			#ifdef __USE_SINGLE_PRECISION__
			options->numRefinementSteps            =  2;
			#else
			options->numRefinementSteps            =  0;
			#endif
	}

	return 0;
}


/*
 *	q p O A S E S _ O p t i o n s _ c o p y
 */
int_t qpOASES_Options_copy(	const qpOASES_Options* const from,
							Options* const to
							)
{
	if ( ( from == 0 ) || ( to == 0 ) )
		return -1;


	to->printLevel                    =  (PrintLevel)(from->printLevel);

	to->enableRamping                 =  (BooleanType)(from->enableRamping);
	to->enableFarBounds               =  (BooleanType)(from->enableFarBounds);
	to->enableFlippingBounds          =  (BooleanType)(from->enableFlippingBounds);
	to->enableRegularisation          =  (BooleanType)(from->enableRegularisation);
	to->enableFullLITests             =  (BooleanType)(from->enableFullLITests);
	to->enableNZCTests                =  (BooleanType)(from->enableNZCTests);
	to->enableDriftCorrection         =  from->enableDriftCorrection;
	to->enableCholeskyRefactorisation =  from->enableCholeskyRefactorisation;
	to->enableEqualities              =  (BooleanType)(from->enableEqualities);

	to->terminationTolerance          =  from->terminationTolerance;
	to->boundTolerance                =  from->boundTolerance;
	to->boundRelaxation               =  from->boundRelaxation;
	to->epsNum                        =  from->epsNum;
	to->epsDen                        =  from->epsDen;
	to->maxPrimalJump                 =  from->maxPrimalJump;
	to->maxDualJump                   =  from->maxDualJump;

	to->initialRamping                =  from->initialRamping;
	to->finalRamping                  =  from->finalRamping;
	to->initialFarBounds              =  from->initialFarBounds;
	to->growFarBounds                 =  from->growFarBounds;
 	to->initialStatusBounds           =  (SubjectToStatus)(from->initialStatusBounds);
	to->epsFlipping                   =  from->epsFlipping;
	to->numRegularisationSteps        =  from->numRegularisationSteps;
	to->epsRegularisation             =  from->epsRegularisation;
	to->numRefinementSteps            =  from->numRefinementSteps;
	to->epsIterRef                    =  from->epsIterRef;
	to->epsLITests                    =  from->epsLITests;
	to->epsNZCTests                   =  from->epsNZCTests;

	to->enableDropInfeasibles         =  (BooleanType)(from->enableDropInfeasibles);
    to->dropBoundPriority             =  from->dropBoundPriority;
    to->dropEqConPriority             =  from->dropEqConPriority;
    to->dropIneqConPriority           =  from->dropIneqConPriority;

	return 0;
}



/*
 *	q p O A S E S _ o b t a i n O u t p u t s
 */
int_t qpOASES_obtainOutputs(	const QProblemB* const globalQpObject,
								returnValue returnvalue,
								real_t* const x,
								real_t* const y,
								real_t* const obj,
								int_t* const status
								)
{
	if ( globalQpObject == 0 )
		return -1;

	/* 代码说明：相关模块实现。 */
	if ( x != 0 ) { globalQpObject->getPrimalSolution( x ); }
	if ( y != 0 ) { globalQpObject->getDualSolution( y ); }
	if ( obj != 0 ) { *obj = globalQpObject->getObjVal( ); }
	*status = getSimpleStatus( returnvalue );

	return 0;
}



/*
 *	Q P r o b l e m _ s e t u p
 */
int_t QProblem_setup(	int_t nV,
						int_t nC,
						int_t hessianType
						)
{
	if ( ( nV < 1 ) || ( nC < 0 ) )
		return -1;

	if ( ( hessianType < 0 ) || ( hessianType > 6 ) )
		return -1;

	if ( QProblem_cleanup() != 0 )
		return -1;

	globalQProblemObject = new QProblem( nV,nC,(HessianType)hessianType );

	return 0;
}


/*
 *	Q P r o b l e m _ i n i t
 */
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
						)
{
	/* abort if QProblem_setup has not been called */
	if ( globalQProblemObject == 0 )
		return -1;

	/* 代码说明：相关模块实现。 */
	if ( options != 0 )
	{
		qpOASES_Options_copy( options,&globalOptionsObject );
		globalQProblemObject->setOptions( globalOptionsObject );
	}

	/* 代码说明：相关模块实现。 */
	returnValue returnvalue = globalQProblemObject->init( H,g,A,lb,ub,lbA,ubA, *nWSR,cputime );

	/* 代码说明：相关模块实现。 */
	return qpOASES_obtainOutputs( globalQProblemObject,returnvalue, x,y,obj,status );
}


/*
 *	Q P r o b l e m _ h o t s t a r t
 */
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
							)
{
	/* abort if QProblem_setup has not been called */
	if ( globalQProblemObject == 0 )
		return -1;

	/* 代码说明：相关模块实现。 */
	returnValue returnvalue = globalQProblemObject->hotstart( g,lb,ub,lbA,ubA, *nWSR,cputime );

	/* 代码说明：相关模块实现。 */
	return qpOASES_obtainOutputs( globalQProblemObject,returnvalue, x,y,obj,status );

	return 0;
}


/*
 *	Q P r o b l e m _ c l e a n u p
 */
int_t QProblem_cleanup( )
{
	if ( globalQProblemObject != 0 )
	{
		delete globalQProblemObject;
		globalQProblemObject = 0;
	}

	return 0;
}



/*
 *	Q P r o b l e m B _ s e t u p
 */
int_t QProblemB_setup(	int_t nV,
						int_t hessianType
						)
{
	if ( nV < 1 )
		return -1;

	if ( ( hessianType < 0 ) || ( hessianType > 6 ) )
		return -1;

	if ( QProblemB_cleanup() != 0 )
		return -1;

	globalQProblemBObject = new QProblemB( nV,(HessianType)hessianType );

	return 0;
}


/*
 *	Q P r o b l e m B _ i n i t
 */
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
						)
{
	/* abort if QProblemB_setup has not been called */
	if ( globalQProblemBObject == 0 )
		return -1;

	/* 代码说明：相关模块实现。 */
	if ( options != 0 )
	{
		qpOASES_Options_copy( options,&globalOptionsObject );
		globalQProblemBObject->setOptions( globalOptionsObject );
	}

	/* 代码说明：相关模块实现。 */
	returnValue returnvalue = globalQProblemBObject->init( H,g,lb,ub, *nWSR,cputime );

	/* 代码说明：相关模块实现。 */
	return qpOASES_obtainOutputs( globalQProblemBObject,returnvalue, x,y,obj,status );
}


/*
 *	Q P r o b l e m B _ h o t s t a r t
 */
int_t QProblemB_hotstart(	const real_t* const g,
							const real_t* const lb,
							const real_t* const ub,
							int_t* const nWSR,
							real_t* const cputime,
							real_t* const x,
							real_t* const y,
							real_t* const obj,
							int_t* const status
							)
{
	/* abort if QProblemB_setup has not been called */
	if ( globalQProblemBObject == 0 )
		return -1;

	/* 代码说明：相关模块实现。 */
	returnValue returnvalue = globalQProblemBObject->hotstart( g,lb,ub, *nWSR,cputime );

	/* 代码说明：相关模块实现。 */
	return qpOASES_obtainOutputs( globalQProblemBObject,returnvalue, x,y,obj,status );

	return 0;
}


/*
 *	Q P r o b l e m B _ c l e a n u p
 */
int_t QProblemB_cleanup( )
{
	if ( globalQProblemBObject != 0 )
	{
		delete globalQProblemBObject;
		globalQProblemBObject = 0;
	}

	return 0;
}



/*
 *	S Q P r o b l e m _ s e t u p
 */
int_t SQProblem_setup(	int_t nV,
						int_t nC,
						int_t hessianType
						)
{
	if ( ( nV < 1 ) || ( nC < 0 ) )
		return -1;

	if ( ( hessianType < 0 ) || ( hessianType > 6 ) )
		return -1;

	if ( SQProblem_cleanup() != 0 )
		return -1;

	globalSQProblemObject = new SQProblem( nV,nC,(HessianType)hessianType );

	return 0;
}


/*
 *	S Q P r o b l e m _ i n i t
 */
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
						)
{
	/* abort if SQProblem_setup has not been called */
	if ( globalSQProblemObject == 0 )
		return -1;

	/* 代码说明：相关模块实现。 */
	if ( options != 0 )
	{
		qpOASES_Options_copy( options,&globalOptionsObject );
		globalSQProblemObject->setOptions( globalOptionsObject );
	}

	/* 代码说明：相关模块实现。 */
	returnValue returnvalue = globalSQProblemObject->init( H,g,A,lb,ub,lbA,ubA, *nWSR,cputime );

	/* 代码说明：相关模块实现。 */
	return qpOASES_obtainOutputs( globalSQProblemObject,returnvalue, x,y,obj,status );
}


/*
 *	S Q P r o b l e m _ h o t s t a r t
 */
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
							)
{
	/* abort if SQProblem_setup has not been called */
	if ( globalSQProblemObject == 0 )
		return -1;

	/* 代码说明：相关模块实现。 */
	returnValue returnvalue = globalSQProblemObject->hotstart( H,g,A,lb,ub,lbA,ubA, *nWSR,cputime );

	/* 代码说明：相关模块实现。 */
	return qpOASES_obtainOutputs( globalSQProblemObject,returnvalue, x,y,obj,status );

	return 0;
}


/*
 *	S Q P r o b l e m _ c l e a n u p
 */
int_t SQProblem_cleanup( )
{
	if ( globalSQProblemObject != 0 )
	{
		delete globalSQProblemObject;
		globalSQProblemObject = 0;
	}

	return 0;
}


/* 代码说明：相关模块实现。 */
