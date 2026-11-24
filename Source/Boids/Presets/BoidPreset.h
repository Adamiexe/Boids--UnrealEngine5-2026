#pragma once
#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "BoidPreset.generated.h"

USTRUCT(BlueprintType)
struct FS_BoidPreset:public FTableRowBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 NumberOfBoids = 100;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Velocity = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float PerceptionRadius = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float FieldOfViewAngle = 120.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float SeparationWeight = 1.5f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float SeparationRadius = 100.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float AlignmentWeight = 1.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float CohesionWeight = 1.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bEnableObstacleAvoidance = true;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float ObstacleAvoidanceWeight = 2.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float ObstacleDetectionDistance = 300.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 NumberOfRaycasts = 8;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 VollumeScale = 5;
	
	FS_BoidPreset()
	{
		NumberOfBoids = 100;
		Velocity = 1000.0f;
		PerceptionRadius = 200.0f;
		FieldOfViewAngle = 120.0f;
		SeparationWeight = 1.5f;
		SeparationRadius = 100.0f;
		AlignmentWeight = 1.0f;
		CohesionWeight = 1.0f;
		bEnableObstacleAvoidance = true;
		ObstacleAvoidanceWeight = 2.0f;
		ObstacleDetectionDistance = 300.0f;
		NumberOfRaycasts = 4;
		VollumeScale = 5;
	}

};