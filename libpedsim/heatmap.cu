#include "ped_model.h"
#include <cstdlib>
#include <iostream>

// Memory leak check with msvc++
#include <stdlib.h>
#include <thread>

using namespace std;

// Constant used in blurrin step.
#define WEIGHTSUM 273

__constant__ int w[5][5] = {
		{ 1, 4, 7, 4, 1 },
		{ 4, 16, 26, 16, 4 },
		{ 7, 26, 41, 26, 7 },
		{ 4, 16, 26, 16, 4 },
		{ 1, 4, 7, 4, 1 }
	};

// Heatmaps on the GPU;
int **device_heatmap;
int **device_scaled_heatmap;
int **device_blurred_heatmap;
int *device_hm;
int *device_shm;
int *device_bhm;

// Sets up the heatmap
void Ped::Model::setupHeatmapCuda()
{
    // Allocate heatmaps for CPU.
    size_t heatmap_size = SIZE*sizeof(int*);
    size_t hm_size = SIZE*SIZE*sizeof(int);
    size_t scaled_map_size = SCALED_SIZE*sizeof(int*);
    size_t shm_size = SCALED_SIZE*SCALED_SIZE*sizeof(int);

	int *hm = (int*)calloc(SIZE*SIZE, sizeof(int));
	int *shm = (int*)malloc(shm_size);
	int *bhm = (int*)malloc(shm_size);
	heatmap = (int**)malloc(heatmap_size);
	scaled_heatmap = (int**)malloc(scaled_map_size);
	blurred_heatmap = (int**)malloc(scaled_map_size);

	// Allocate heatmaps for GPU.
	cudaMalloc((void **)&device_hm, hm_size);
	cudaMemset((void *)device_hm, 0, hm_size);
	cudaMalloc((void **)&device_shm, shm_size);
	cudaMalloc((void **)&device_bhm, shm_size);
	// Use global GPU heatmaps defined in outer scope.
	cudaMalloc((void ***)&device_heatmap, heatmap_size);
	cudaMalloc((void ***)&device_scaled_heatmap, scaled_map_size);
	cudaMalloc((void ***)&device_blurred_heatmap, scaled_map_size);

	// Set up 2d array on CPU.
	for (int i = 0; i < SIZE; i++)
	{
		heatmap[i] = hm + SIZE*i;
	}
	for (int i = 0; i < SCALED_SIZE; i++)
	{
		scaled_heatmap[i] = shm + SCALED_SIZE*i;
		blurred_heatmap[i] = bhm + SCALED_SIZE*i;
	}

	// Set up 2d array on GPU.
	for (int i = 0; i < SIZE; i++)
	{
	    cudaMemcpy((void *)device_heatmap, device_hm + SIZE*i, sizeof(int*), cudaMemcpyDeviceToDevice);
	}
	for (int i = 0; i < SCALED_SIZE; i++)
	{
	    cudaMemcpy((void *)device_scaled_heatmap, device_shm + SCALED_SIZE*i, sizeof(int*), cudaMemcpyDeviceToDevice);
		cudaMemcpy((void *)device_blurred_heatmap, device_bhm + SCALED_SIZE*i, sizeof(int*), cudaMemcpyDeviceToDevice);
	}
}

__global__ void fadeHeatmap(int *heatmap)
{
    int id = blockIdx.x*blockDim.x+threadIdx.x;
    
    if(id <SIZE*SIZE){
        heatmap[id] = (int)round(heatmap[id] * 0.80);
    }
    /* 
    for (int x = 0; x < SIZE; x++)
	{
		for (int y = 0; y < SIZE; y++)
		{
			// heat fades
			heatmap[y][x] = (int)round(heatmap[y][x] * 0.80);
		}
	}
	*/
}

__global__ void sumHeatmap(int *heatmap, int *agentsDesiredX, int *agentsDesiredY, int agentCount)
{
    int id = blockIdx.x*blockDim.x+threadIdx.x;
    
    if(id < agentCount)
    {
        int x = agentsDesiredX[id];
		int y = agentsDesiredY[id];
      
		if (!(x < 0 || x >= SIZE || y < 0 || y >= SIZE))
		{
		    atomicAdd((int *)heatmap + x*SIZE + y, (int)40);
		}
    }
    /* 
    for (int i = 0; i < agentCount; i++)
	{
		int x = agentsDesiredX[i];
		int y = agentsDesiredY[i];

		if (x < 0 || x >= SIZE || y < 0 || y >= SIZE)
		{
			continue;
		}

		// intensify heat for better color results
		heatmap[y][x] += 40;
	}

	for (int x = 0; x < SIZE; x++)
	{
		for (int y = 0; y < SIZE; y++)
		{
			heatmap[y][x] = heatmap[y][x] < 255 ? heatmap[y][x] : 255;
		}
	}
	*/
}

__global__ void scaleHeatmap(int **scaled_heatmap, int **heatmap)
{
    int id = blockIdx.x*blockDim.x+threadIdx.x;
    
    if(id <SCALED_SIZE*SCALED_SIZE){
        scaled_heatmap[id] = heatmap[(int)floor((double)(id/CELLSIZE))];
    }
        /* 
    for (int y = 0; y < SIZE; y++)
	{
		for (int x = 0; x < SIZE; x++)
		{
			int value = heatmap[y][x];
			for (int cellY = 0; cellY < CELLSIZE; cellY++)
			{
				for (int cellX = 0; cellX < CELLSIZE; cellX++)
				{
					scaled_heatmap[y * CELLSIZE + cellY][x * CELLSIZE + cellX] = value;
				}
			}
		}
	}
	*/
}

__global__ void blurHeatmap(int *blurred_heatmap, int *scaled_heatmap)
{
    int id = blockIdx.x*blockDim.x+threadIdx.x;
    __shared__ int scaled_heatmap_block[128][128]; //blocksize
    
    scaled_heatmap_block[blockIdx.x + threadIdx.x][blockIdx.y + threadIdx.y] = scaled_heatmap[id];
    
    __syncthreads();
    
    if(id < SCALED_SIZE*SCALED_SIZE){
        int sum = 0;
        for (int k = -2; k < 3; k++)
		{
			for (int l = -2; l < 3; l++)
			{
				sum += w[2 + k][2 + l] * scaled_heatmap_block[blockIdx.x + threadIdx.x+k][blockIdx.y + threadIdx.y + l];
			}
		}
		int value = sum / WEIGHTSUM;
		blurred_heatmap[id] = 0x00FF0000 | value << 24;
    }
    
    /* 
    for (int i = 2; i < SCALED_SIZE - 2; i++)
	{
		for (int j = 2; j < SCALED_SIZE - 2; j++)
		{
			int sum = 0;
			for (int k = -2; k < 3; k++)
			{
				for (int l = -2; l < 3; l++)
				{
					sum += w[2 + k][2 + l] * scaled_heatmap[i + k][j + l];
				}
			}
			int value = sum / WEIGHTSUM;
			blurred_heatmap[i][j] = 0x00FF0000 | value << 24;
		}
	}
	*/
}

void Ped::Model::updateHeatmapCuda(int *agentsDesiredX, int *agentsDesiredY, int agentCount)
{
    // Part 1 of algorithm.
    int blocksize, gridsize;
    blocksize = 128;
    gridsize = ceil(SIZE*SIZE/blocksize);
    fadeHeatmap<<<gridsize, blocksize>>>(device_hm);

    // Part 2 of algorithm.
	// Count how many agents want to go to each location
	sumHeatmap<<<gridsize, blocksize>>>(device_hm, agentsDesiredX, agentsDesiredY, agentCount);

	// Part 3 of algorithm.
	// Scale the data for visual representation
	gridsize = ceil(SCALED_SIZE*SCALED_SIZE/blocksize);
	scaleHeatmap<<<gridsize, blocksize>>>(&device_shm, &device_hm);

	// Part 4 of algorithm.
	// Apply gaussian blurfilter
	blurHeatmap<<<gridsize, blocksize>>>(device_bhm, device_shm);

	// Copy results back from GPU to CPU.
	cudaMemcpy(heatmap, device_heatmap, SIZE*sizeof(int*), cudaMemcpyDeviceToHost);
	cudaMemcpy(scaled_heatmap, device_scaled_heatmap, SCALED_SIZE*sizeof(int*), cudaMemcpyDeviceToHost);
	cudaMemcpy(blurred_heatmap, device_blurred_heatmap, SCALED_SIZE*sizeof(int*), cudaMemcpyDeviceToHost);
}
