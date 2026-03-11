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
cudaEvent_t start_sum, stop_sum, start_scale, stop_scale, start_blur, stop_blur;
int *device_agentsDesiredX;
int *device_agentsDesiredY;

// Sets up the heatmap
void Ped::Model::setupHeatmapCuda()
{
	cudaMalloc((void **)&device_agentsDesiredX, agents.size()*sizeof(int));
	cudaMalloc((void **)&device_agentsDesiredY, agents.size()*sizeof(int));
	cudaEventCreate(&start_sum);
	cudaEventCreate(&stop_sum);
	cudaEventCreate(&start_scale);
	cudaEventCreate(&stop_scale);
	cudaEventCreate(&start_blur);
	cudaEventCreate(&stop_blur);
	sum_seconds = 0;
	scale_seconds = 0;
	blur_seconds = 0;

    // Allocate heatmaps for CPU.
    size_t heatmap_size = SIZE*sizeof(int*);
    size_t hm_size = SIZE*SIZE*sizeof(int);
    size_t scaled_map_size = SCALED_SIZE*sizeof(int*);
    size_t shm_size = SCALED_SIZE*SCALED_SIZE*sizeof(int);

	hm = (int*)calloc(SIZE*SIZE, sizeof(int));
	shm = (int*)malloc(shm_size);
	bhm = (int*)malloc(shm_size);
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
		    atomicAdd((int *)heatmap+y*SIZE + x, 40);
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
	*/
	/*

	for (int x = 0; x < SIZE; x++)
	{
		for (int y = 0; y < SIZE; y++)
		{
			heatmap[y][x] = heatmap[y][x] < 255 ? heatmap[y][x] : 255;
		}
	}
	*/
}

__global__ void fixHeatmap(int *heatmap){
	int id = blockIdx.x*blockDim.x+threadIdx.x;
    
    if(id < SIZE*SIZE){
		if(!(heatmap[id] < 255)){
			heatmap[id] = 255;
		}
	}
/*

	for (int x = 0; x < SIZE; x++)
	{
		for (int y = 0; y < SIZE; y++)
		{
			heatmap[y][x] = heatmap[y][x] < 255 ? heatmap[y][x] : 255;
		}
	}
	*/
}

__global__ void scaleHeatmap(int *scaled_heatmap, int *heatmap)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < SCALED_SIZE || y < SCALED_SIZE){
    	scaled_heatmap[y * SCALED_SIZE + x] = heatmap[y/CELLSIZE * SIZE + x/CELLSIZE];
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
	/*
	int id = blockIdx.x*blockDim.x+threadIdx.x;
    
    if(id < SCALED_SIZE*SCALED_SIZE){
        int sum = 0;
        for (int k = -2; k < 3; k++)
		{
			for (int l = -2; l < 3; l++)
			{
				sum += w[2 + k][2 + l] * scaled_heatmap[id];
			}
		}
		int value = sum / WEIGHTSUM;
		blurred_heatmap[id] = 0x00FF0000 | value << 24;
    }
*/

	
    int idx = blockIdx.x*blockDim.x+threadIdx.x;
	int idy = blockIdx.y*blockDim.y+threadIdx.y;
	int id = idx + idy*SCALED_SIZE;
    __shared__ int scaled_heatmap_block[36][36]; //blocksize + 4

	if (idx < SCALED_SIZE && idy < SCALED_SIZE) {
    	scaled_heatmap_block[threadIdx.x][threadIdx.y] = scaled_heatmap[id];
	}

	__syncthreads();
    
    if(id < SCALED_SIZE*SCALED_SIZE){
        int sum = 0;
        for (int k = -2; k < 3; k++)
		{
			for (int l = -2; l < 3; l++)
			{
				if(0 <= ((int) threadIdx.x+k) && 0 <= ((int)threadIdx.y + l)){
					sum += w[2 + k][2 + l] * scaled_heatmap_block[threadIdx.x+k][threadIdx.y + l];
				}
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
	cudaMemcpyAsync(device_agentsDesiredX, agentsDesiredX, agentCount*sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpyAsync(device_agentsDesiredY, agentsDesiredY, agentCount*sizeof(int), cudaMemcpyHostToDevice);

    // Part 1 of algorithm.
    int blocksize, gridsize;
    blocksize = 256;
    gridsize = (SIZE*SIZE+blocksize-1)/blocksize;
    fadeHeatmap<<<gridsize, blocksize>>>(device_hm);

    // Part 2 of algorithm.
	// Count how many agents want to go to each location
	gridsize = (agentCount+blocksize-1)/blocksize;
	cudaEventRecord(start_sum);
	sumHeatmap<<<gridsize, blocksize>>>(device_hm, device_agentsDesiredX, device_agentsDesiredY, agentCount);
	
	cudaEventRecord(stop_sum);
	cudaEventSynchronize(stop_sum);
	cudaEventElapsedTime(&sum_seconds_new, start_sum, stop_sum);
	sum_seconds = sum_seconds + sum_seconds_new;
	

	gridsize = (SIZE*SIZE+blocksize-1)/blocksize;
	cudaEventRecord(start_sum);
	fixHeatmap<<<gridsize, blocksize>>>(device_hm);
	cudaEventRecord(stop_sum);
	cudaEventSynchronize(stop_sum);
	cudaEventElapsedTime(&sum_seconds_new, start_sum, stop_sum);
	sum_seconds = sum_seconds + sum_seconds_new;
	
	

	// Part 3 of algorithm.
	// Scale the data for visual representation
	dim3 block(32, 32); //warp size
	dim3 grid((SCALED_SIZE + block.x - 1) / block.x, (SCALED_SIZE + block.y - 1) / block.y);
	cudaEventRecord(start_scale);
	scaleHeatmap<<<grid, block>>>(device_shm, device_hm);
	
	cudaEventRecord(stop_scale);
	cudaEventSynchronize(stop_scale);
	cudaEventElapsedTime(&scale_seconds_new, start_scale, stop_scale);
	scale_seconds = scale_seconds + scale_seconds_new;
	

	// Part 4 of algorithm.
	// Apply gaussian blurfilter
	cudaEventRecord(start_blur);
	blurHeatmap<<<grid, block>>>(device_bhm, device_shm);
	cudaEventRecord(stop_blur);
	cudaEventSynchronize(stop_blur);
	cudaEventElapsedTime(&blur_seconds_new, start_blur, stop_blur);
	blur_seconds = blur_seconds + blur_seconds_new;
	
	
	cudaMemcpyAsync(bhm, device_bhm, SCALED_SIZE*SCALED_SIZE*sizeof(int), cudaMemcpyDeviceToHost);
}
