#pragma once

#include "CoreMinimal.h"

/**
 * @brief High-performance 3D Spatial Grid using a flat linked-list approach.
 * @details Allocates zero memory per frame. Employs a Head/Next array pair
 *          to index boids into 3D grid cells, drastically accelerating neighbor lookups
 *          from O(N^2) to near O(N).
 */
class FSpatialHashGrid
{
public:
	FSpatialHashGrid() = default;

	/**
	 * @brief Configures grid dimensions based on simulation bounds and cell size.
	 * 
	 * @param BoxCenter Center of the bounding box
	 * @param BoxExtent Half-size of the bounding box
	 * @param InCellSize Size of each cubic cell (ideally >= PerceptionRadius)
	 * @param NumBoids Total number of boids to reserve capacity for
	 */
	void Initialize(const FVector& BoxCenter, const FVector& BoxExtent, const float InCellSize, const int32 NumBoids)
	{
		CellSize = FMath::Max(1.0f, InCellSize);
		InvCellSize = 1.0f / CellSize;
		MinBounds = BoxCenter - BoxExtent;
		MaxBounds = BoxCenter + BoxExtent;

		const FVector TotalSize = BoxExtent * 2.0f;
		Dimensions.X = FMath::Max(1, FMath::CeilToInt(TotalSize.X * InvCellSize));
		Dimensions.Y = FMath::Max(1, FMath::CeilToInt(TotalSize.Y * InvCellSize));
		Dimensions.Z = FMath::Max(1, FMath::CeilToInt(TotalSize.Z * InvCellSize));

		const int32 TotalCells = Dimensions.X * Dimensions.Y * Dimensions.Z;
		CellHead.Init(INDEX_NONE, TotalCells);
		BoidNext.SetNumUninitialized(NumBoids);
	}

	/**
	 * @brief Clears the grid for a new frame without reallocating memory.
	 */
	FORCEINLINE void Clear()
	{
		if (CellHead.Num() > 0)
		{
			FMemory::Memset(CellHead.GetData(), 0xFF, CellHead.Num() * sizeof(int32));
		}
	}

	/**
	 * @brief Ensures the BoidNext array can accommodate the given number of boids.
	 */
	FORCEINLINE void PrepareBoids(const int32 NumBoids)
	{
		if (BoidNext.Num() != NumBoids)
		{
			BoidNext.SetNumUninitialized(NumBoids);
		}
	}

	/**
	 * @brief Inserts a boid into the grid.
	 * 
	 * @param BoidIndex Index of the boid (0 to NumBoids - 1)
	 * @param Position 3D world position of the boid
	 */
	FORCEINLINE void Insert(const int32 BoidIndex, const FVector& Position)
	{
		const FIntVector Coord = GetCellCoords(Position);
		const int32 CellIdx = ToCellIndex(Coord);

		BoidNext[BoidIndex] = CellHead[CellIdx];
		CellHead[CellIdx] = BoidIndex;
	}

	/**
	 * @brief Converts a 3D world position to cell coordinates.
	 */
	FORCEINLINE FIntVector GetCellCoords(const FVector& Position) const
	{
		const FVector Rel = Position - MinBounds;
		const int32 X = FMath::Clamp(FMath::FloorToInt(Rel.X * InvCellSize), 0, Dimensions.X - 1);
		const int32 Y = FMath::Clamp(FMath::FloorToInt(Rel.Y * InvCellSize), 0, Dimensions.Y - 1);
		const int32 Z = FMath::Clamp(FMath::FloorToInt(Rel.Z * InvCellSize), 0, Dimensions.Z - 1);
		return FIntVector(X, Y, Z);
	}

	/**
	 * @brief Converts 3D cell coordinates to a flat 1D array index.
	 */
	FORCEINLINE int32 ToCellIndex(const FIntVector& Coord) const
	{
		return Coord.X + (Coord.Y * Dimensions.X) + (Coord.Z * Dimensions.X * Dimensions.Y);
	}

	/**
	 * @brief Queries all boids present in the cells overlapping a sphere.
	 * @details Safe to call concurrently from multiple threads (read-only grid).
	 * 
	 * @tparam CallbackType Callable with signature: bool(int32 NeighborIndex) or void(int32 NeighborIndex)
	 * @param Center Center of the query sphere
	 * @param Radius Radius of the query sphere
	 * @param Callback Invoked for every boid found in candidate cells
	 */
	template <typename CallbackType>
	FORCEINLINE void QuerySphere(const FVector& Center, const float Radius, CallbackType&& Callback) const
	{
		const FIntVector MinCoord = GetCellCoords(Center - FVector(Radius));
		const FIntVector MaxCoord = GetCellCoords(Center + FVector(Radius));

		for (int32 Z = MinCoord.Z; Z <= MaxCoord.Z; ++Z)
		{
			const int32 ZOffset = Z * Dimensions.X * Dimensions.Y;
			for (int32 Y = MinCoord.Y; Y <= MaxCoord.Y; ++Y)
			{
				const int32 YOffset = Y * Dimensions.X;
				for (int32 X = MinCoord.X; X <= MaxCoord.X; ++X)
				{
					const int32 CellIdx = X + YOffset + ZOffset;
					int32 Curr = CellHead[CellIdx];

					while (Curr != INDEX_NONE)
					{
						Callback(Curr);
						Curr = BoidNext[Curr];
					}
				}
			}
		}
	}

	FORCEINLINE float GetCellSize() const { return CellSize; }
	FORCEINLINE const FIntVector& GetDimensions() const { return Dimensions; }
	FORCEINLINE int32 GetTotalCells() const { return CellHead.Num(); }
	FORCEINLINE const FVector& GetMinBounds() const { return MinBounds; }
	FORCEINLINE const FVector& GetMaxBounds() const { return MaxBounds; }
	FORCEINLINE bool IsCellOccupied(const int32 CellIdx) const { return CellHead.IsValidIndex(CellIdx) && CellHead[CellIdx] != INDEX_NONE; }

	FORCEINLINE FVector GetCellCenter(const FIntVector& Coord) const
	{
		const float HalfSize = CellSize * 0.5f;
		return MinBounds + FVector(
			Coord.X * CellSize + HalfSize,
			Coord.Y * CellSize + HalfSize,
			Coord.Z * CellSize + HalfSize
		);
	}

	FORCEINLINE FVector GetCellExtent() const
	{
		const float HalfSize = CellSize * 0.5f;
		return FVector(HalfSize, HalfSize, HalfSize);
	}

private:
	float CellSize = 100.0f;
	float InvCellSize = 0.01f;
	FVector MinBounds = FVector::ZeroVector;
	FVector MaxBounds = FVector::ZeroVector;
	FIntVector Dimensions = FIntVector(1, 1, 1);

	// CellHead[CellIdx] contains the first boid index in the cell, or INDEX_NONE (-1)
	TArray<int32> CellHead;

	// BoidNext[BoidIdx] points to the next boid in the same cell, or INDEX_NONE (-1)
	TArray<int32> BoidNext;
};
