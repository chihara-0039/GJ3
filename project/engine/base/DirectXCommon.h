#pragma once
#include "WinApp.h"
#include <array>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <dxcapi.h>
#include <wrl.h>
#include <string>
#include <chrono>
#include <cstdint>
#include "effect/RenderTexture.h"
#include "math/Struct.h"
#include "externals/DirectXTex/DirectXTex.h"

/**
 * DirectXCommon繧ｯ繝ｩ繧ｹ
 * DirectX12縺ｮ蝓ｺ逶､邂｡逅・↓蜉縺医√お繝・ぅ繧ｿ逕ｨ縺ｮ縲後Ξ繝ｳ繝繝ｼ繝・け繧ｹ繝√Ε縲咲ｮ｡逅・ｩ溯・繧定ｿｽ蜉縲・
 */
class DirectXCommon {
public:
    enum class FullscreenPostEffectType {
        Copy = 0,
        Grayscale,
        Sepia,
        Blur,
        Bloom,
        BoxFilter3x3,
        BoxFilter5x5,
        RadialBlur,
        Dissolve,
        OutlineLuminance,
        OutlineDepth,
        OutlineNormal,
        OutlineDepthNormal,
        Vignette,
        RandomNoise,
        HSVFilter,
        LinearToSRGB,
        Count,
    };

    struct FullscreenPostEffectParameter {
        float grayscaleIntensity = 1.0f;
        float sepiaIntensity = 1.0f;
        float blurStrength = 1.0f;
        float padding = 0.0f;
        float bloomThreshold = 0.65f;
        float bloomIntensity = 1.0f;
        float bloomRadius = 6.0f;
        float bloomSoftKnee = 0.2f;
        float outlineThreshold = 0.15f;
        float outlineIntensity = 1.0f;
        float outlineThickness = 1.0f;
        float outlinePadding = 0.0f;
        float depthOutlineThreshold = 0.002f;
        float depthOutlineIntensity = 1.0f;
        float depthOutlineThickness = 1.0f;
        float depthOutlinePadding = 0.0f;
        float depthOutlineNearClip = 0.1f;
        float depthOutlineFarClip = 1000.0f;
        float depthOutlineLinearize = 1.0f;
        float depthOutlineLinearPadding = 0.0f;
        float normalOutlineThreshold = 0.25f;
        float normalOutlineIntensity = 1.0f;
        float normalOutlineThickness = 1.0f;
        float normalOutlinePadding = 0.0f;
    };
    static_assert(sizeof(FullscreenPostEffectParameter) == 96);

    struct VignetteParamForGPU {
        float scale = 16.0f;
        float power = 0.8f;
        float intensity = 1.0f;
        float padding = 0.0f;
    };
    static_assert(sizeof(VignetteParamForGPU) == 16);

    struct RadialBlurParamForGPU {
        Vector2 center = { 0.5f, 0.5f };
        float blurWidth = 0.01f;
        float intensity = 1.0f;
        int32_t sampleCount = 10;
        float padding[3] = {};
    };
    static_assert(sizeof(RadialBlurParamForGPU) == 32);

    struct DissolveParamForGPU {
        float threshold = 0.5f;
        float edgeWidth = 0.03f;
        float edgeIntensity = 1.0f;
        float enableEdge = 1.0f;
        Vector3 edgeColor = { 1.0f, 0.4f, 0.3f };
        float padding0 = 0.0f;
    };
    static_assert(sizeof(DissolveParamForGPU) == 32);

    struct RandomNoiseParamForGPU {
        float time = 0.0f;
        float strength = 0.2f;
        float scale = 800.0f;
        float mode = 1.0f;
        float animate = 1.0f;
        float padding0 = 0.0f;
        float padding1 = 0.0f;
        float padding2 = 0.0f;
    };
    static_assert(sizeof(RandomNoiseParamForGPU) == 32);

    struct HSVFilterParamForGPU {
        float hue = 0.0f;
        float saturation = 0.0f;
        float value = 0.0f;
        float padding = 0.0f;
    };
    static_assert(sizeof(HSVFilterParamForGPU) == 16);

    void Initialize(WinApp* winApp);

    // --- 謠冗判繝輔Ο繝ｼ邂｡逅・---
    // 1. 繧ｲ繝ｼ繝逕ｻ髱｢・医ユ繧ｯ繧ｹ繝√Ε・峨∈縺ｮ謠冗判繧帝幕蟋九☆繧・
    void PreDraw();
    void RestoreRenderTextureToRenderTarget();
    void TransitionRenderTexture(D3D12_RESOURCE_STATES stateAfter);
    void TransitionPostEffectResult(D3D12_RESOURCE_STATES stateAfter);
    void TransitionFinalDisplayTexture(D3D12_RESOURCE_STATES stateAfter);
    void TransitionNormalTexture(D3D12_RESOURCE_STATES stateAfter);
    void TransitionDepthBuffer(D3D12_RESOURCE_STATES stateAfter);
    void BeginPostEffectResultRenderTarget();
    void BeginFinalDisplayRenderTarget();
    void BeginSwapChainRenderTarget();
    D3D12_GPU_VIRTUAL_ADDRESS GetCommonPostEffectParameterAddress() const;
    D3D12_GPU_VIRTUAL_ADDRESS GetVignetteParameterAddress() const;
    D3D12_GPU_VIRTUAL_ADDRESS GetRadialBlurParameterAddress() const;
    D3D12_GPU_VIRTUAL_ADDRESS GetDissolveParameterAddress() const;
    D3D12_GPU_VIRTUAL_ADDRESS GetRandomNoiseParameterAddress() const;
    D3D12_GPU_VIRTUAL_ADDRESS GetHSVFilterParameterAddress() const;
    void SetFullscreenPostEffectParameter(const FullscreenPostEffectParameter& parameter);
    void SetVignetteParameter(const VignetteParamForGPU& parameter);
    void SetRadialBlurParameter(const RadialBlurParamForGPU& parameter);
    void SetDissolveParameter(const DissolveParamForGPU& parameter);
    void SetRandomNoiseParameter(const RandomNoiseParamForGPU& parameter);
    void SetHSVFilterParameter(const HSVFilterParamForGPU& parameter);
    void ResizeSwapChainIfNeeded();
    // 3. 蜈ｨ縺ｦ縺ｮ謠冗判繧堤ｵゆｺ・＠縲∫判髱｢繧定｡ｨ遉ｺ縺吶ｋ
    void PostDraw();

    // --- 繧ｲ繝・ち繝ｼ ---
    ID3D12Device* GetDevice() const { return device_.Get(); }
    ID3D12GraphicsCommandList* GetCommandList() const { return commandList_.Get(); }
    ID3D12CommandQueue* GetCommandQueue() const { return commandQueue_.Get(); }
    size_t GetSwapChainResourcesNum() const { return backBuffers_.size(); }

    // ImGui縺ｫ貂｡縺吶◆繧√・縲∵緒縺崎ｾｼ縺ｿ貂医∩繝・け繧ｹ繝√Ε繝ｪ繧ｽ繝ｼ繧ｹ繧貞叙蠕・
    ID3D12Resource* GetRenderTextureResource() const { return sceneRenderTexture_.GetResource(); }
    ID3D12Resource* GetPostEffectResultResource() const { return postEffectResultTexture_.GetResource(); }
    ID3D12Resource* GetFinalDisplayTextureResource() const { return finalDisplayTexture_.GetResource(); }
    ID3D12Resource* GetDepthBufferResource() const { return depthBuffer_.Get(); }
    ID3D12Resource* GetNormalTextureResource() const { return normalTexture_.GetResource(); }
    void SetSceneRenderTarget();
    void SetSceneRenderTargetsWithNormal();

    // --- 蜷・ｨｮ繝倥Ν繝代・ ---
    DirectX::ScratchImage LoadTexture(const std::string& filePath);
    Microsoft::WRL::ComPtr<ID3D12Resource> CreateTextureResource(const DirectX::TexMetadata& metadata);
    void UploadTextureData(ID3D12Resource* texture, const DirectX::ScratchImage& mipImages);
    Microsoft::WRL::ComPtr<IDxcBlob> CompileShader(const std::wstring& filePath, const std::wstring& profile);
    Microsoft::WRL::ComPtr<ID3D12Resource> CreateBufferResource(size_t sizeInBytes);
    Microsoft::WRL::ComPtr<ID3D12Resource> CreateUAVBufferResource(size_t sizeInBytes, D3D12_RESOURCE_STATES initialState);
    Microsoft::WRL::ComPtr<ID3D12Resource> CreateRenderTextureResource(uint32_t width, uint32_t height, DXGI_FORMAT format, const float* clearColor);
    uint32_t AllocateRTV();
    D3D12_CPU_DESCRIPTOR_HANDLE GetRTVHandle(uint32_t index) const;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> CreateDescriptorHeap(D3D12_DESCRIPTOR_HEAP_TYPE type, UINT numDescriptors, bool shaderVisible);

private:
    void InitializeDevice();
    void InitializeCommand();
    void InitializeSwapChain(WinApp* winApp);
    void InitializeRenderTargetView(); // RTV繝偵・繝励・繧ｵ繧､繧ｺ繧呈僑蠑ｵ縺励∪縺・
    void InitializeDepthStencilView();
    void InitializeRenderTexture();    // 繧ｲ繝ｼ繝逕ｻ髱｢逕ｨ縺ｮ繝・け繧ｹ繝√Ε繧剃ｽ懈・
    void InitializeFullscreenPostEffectParameter();
    void InitializeFence();
    void InitializeDXCCompiler();
    void InitializeFixFPS();
    void UpdateFixFPS();
    void SetFullscreenViewportAndScissor();
    void SetSwapChainViewportAndScissor();

    WinApp* winApp_ = nullptr;
    Microsoft::WRL::ComPtr<ID3D12Device> device_;
    Microsoft::WRL::ComPtr<IDXGIFactory7> dxgiFactory_;
    Microsoft::WRL::ComPtr<ID3D12CommandQueue> commandQueue_;
    Microsoft::WRL::ComPtr<ID3D12CommandAllocator> commandAllocator_;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> commandList_;
    Microsoft::WRL::ComPtr<IDXGISwapChain4> swapChain_;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, 2> backBuffers_;
    uint32_t swapChainWidth_ = 0;
    uint32_t swapChainHeight_ = 0;

    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> rtvHeap_;
    UINT rtvDescriptorSize_ = 0;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> dsvHeap_;
    Microsoft::WRL::ComPtr<ID3D12Resource> depthBuffer_;
    D3D12_RESOURCE_STATES depthBufferState_ = D3D12_RESOURCE_STATE_DEPTH_WRITE;

    // --- 霑ｽ蜉・壹ご繝ｼ繝逕ｻ髱｢繧剃ｿ晏ｭ倥☆繧九ユ繧ｯ繧ｹ繝√Ε ---
    RenderTexture sceneRenderTexture_;
    RenderTexture postEffectResultTexture_;
    RenderTexture finalDisplayTexture_;
    RenderTexture normalTexture_;
    uint32_t nextRtvIndex_ = 2;

    static constexpr DXGI_FORMAT kRenderTargetFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    static constexpr DXGI_FORMAT kNormalTextureFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    float renderTextureClearColor_[4] = { 0.10f, 0.25f, 0.50f, 1.0f };

    Microsoft::WRL::ComPtr<ID3D12Resource> fullscreenPostEffectParameterResource_;
    FullscreenPostEffectParameter* mappedFullscreenPostEffectParameter_ = nullptr;
    Microsoft::WRL::ComPtr<ID3D12Resource> vignetteParameterResource_;
    VignetteParamForGPU* mappedVignetteParameter_ = nullptr;
    Microsoft::WRL::ComPtr<ID3D12Resource> radialBlurParameterResource_;
    RadialBlurParamForGPU* mappedRadialBlurParameter_ = nullptr;
    Microsoft::WRL::ComPtr<ID3D12Resource> dissolveParameterResource_;
    DissolveParamForGPU* mappedDissolveParameter_ = nullptr;
    Microsoft::WRL::ComPtr<ID3D12Resource> randomNoiseParameterResource_;
    RandomNoiseParamForGPU* mappedRandomNoiseParameter_ = nullptr;
    Microsoft::WRL::ComPtr<ID3D12Resource> hsvFilterParameterResource_;
    HSVFilterParamForGPU* mappedHSVFilterParameter_ = nullptr;

    Microsoft::WRL::ComPtr<ID3D12Fence> fence_;
    uint64_t fenceValue_ = 0;
    HANDLE fenceEvent_ = {};
    Microsoft::WRL::ComPtr<IDxcUtils> dxcUtils_;
    Microsoft::WRL::ComPtr<IDxcCompiler3> dxcCompiler_;
    Microsoft::WRL::ComPtr<IDxcIncludeHandler> includeHandler_;
    std::chrono::steady_clock::time_point reference_;
};
