#include "PostProcess.h"
#include "base/Logger.h"
#include "base/SrvManager.h"
#include <cassert>

// =========================================================
// 初期化（動作確認済みの色反転版と同一構造）
// =========================================================
void PostProcess::Initialize(DirectXCommon* dxCommon, SrvManager* srvManager) {
    assert(dxCommon);
    assert(srvManager);
    dxCommon_ = dxCommon;
    srvManager_ = srvManager;

    // --- 1. 最終出力テクスチャを1枚作る ---
    float clearColor[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    texOutput_ = dxCommon_->CreateRenderTextureResource(1280, 720, DXGI_FORMAT_R8G8B8A8_UNORM, clearColor);

    outputRtvIndex_ = dxCommon_->AllocateRTV();
    dxCommon_->GetDevice()->CreateRenderTargetView(
        texOutput_.Get(), nullptr,
        dxCommon_->GetRTVHandle(outputRtvIndex_)
    );

    outputSrvIndex_ = srvManager_->Allocate();
    srvManager_->CreateSRVforTexture2D(outputSrvIndex_, texOutput_.Get(), DXGI_FORMAT_R8G8B8A8_UNORM, 1);

    // --- 2. 定数バッファ作成 ---
    cbBloom_ = dxCommon_->CreateBufferResource(sizeof(BloomParam));
    UpdateBlurWeights(); // 初期値をGPUに転送

    // --- 3. ルートシグネチャ: [0]=SRV(t0)  [1]=CBV(b0) ---
    D3D12_DESCRIPTOR_RANGE range{};
    range.RangeType                         = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    range.NumDescriptors                    = 1;
    range.BaseShaderRegister                = 0;
    range.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    D3D12_ROOT_PARAMETER params[2]{};
    // [0] SRVテーブル (t0)
    params[0].ParameterType                       = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[0].DescriptorTable.pDescriptorRanges   = &range;
    params[0].DescriptorTable.NumDescriptorRanges = 1;
    params[0].ShaderVisibility                    = D3D12_SHADER_VISIBILITY_PIXEL;
    // [1] CBV直接参照 (b0)
    params[1].ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    params[1].Descriptor.ShaderRegister = 0;
    params[1].ShaderVisibility          = D3D12_SHADER_VISIBILITY_PIXEL;

    D3D12_STATIC_SAMPLER_DESC sampler{};
    sampler.Filter           = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    sampler.AddressU         = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressV         = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressW         = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.ComparisonFunc   = D3D12_COMPARISON_FUNC_NEVER;
    sampler.MaxLOD           = D3D12_FLOAT32_MAX;
    sampler.ShaderRegister   = 0;
    sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

    D3D12_ROOT_SIGNATURE_DESC rsDesc{};
    rsDesc.NumParameters     = 2;
    rsDesc.pParameters       = params;
    rsDesc.NumStaticSamplers = 1;
    rsDesc.pStaticSamplers   = &sampler;
    rsDesc.Flags             = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

    Microsoft::WRL::ComPtr<ID3DBlob> blob, err;
    D3D12SerializeRootSignature(&rsDesc, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &err);
    dxCommon_->GetDevice()->CreateRootSignature(
        0, blob->GetBufferPointer(), blob->GetBufferSize(),
        IID_PPV_ARGS(&rootSignature_)
    );

    // --- 4. パイプラインステート ---
    auto vsBlob = dxCommon_->CompileShader(L"Resources/shader/PostProcess.VS.hlsl", L"vs_6_0");
    auto psBlob = dxCommon_->CompileShader(L"Resources/shader/SinglePassBloom.PS.hlsl", L"ps_6_0");
    if (vsBlob == nullptr) {
        Logger::Log("PostProcess::Initialize failed: vertex shader compile failed.\n");
        assert(false);
        return;
    }
    if (psBlob == nullptr) {
        Logger::Log("PostProcess::Initialize failed: pixel shader compile failed.\n");
        assert(false);
        return;
    }

    D3D12_BLEND_DESC blend{};
    blend.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

    D3D12_RASTERIZER_DESC raster{};
    raster.CullMode = D3D12_CULL_MODE_NONE;
    raster.FillMode = D3D12_FILL_MODE_SOLID;

    D3D12_DEPTH_STENCIL_DESC ds{};
    ds.DepthEnable = false;

    D3D12_GRAPHICS_PIPELINE_STATE_DESC pso{};
    pso.InputLayout           = { nullptr, 0 };
    pso.pRootSignature        = rootSignature_.Get();
    pso.VS                    = { vsBlob->GetBufferPointer(), vsBlob->GetBufferSize() };
    pso.PS                    = { psBlob->GetBufferPointer(), psBlob->GetBufferSize() };
    pso.BlendState            = blend;
    pso.RasterizerState       = raster;
    pso.DepthStencilState     = ds;
    pso.DSVFormat             = DXGI_FORMAT_UNKNOWN;
    pso.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    pso.NumRenderTargets      = 1;
    pso.RTVFormats[0]         = DXGI_FORMAT_R8G8B8A8_UNORM;
    pso.SampleDesc.Count      = 1;
    pso.SampleMask            = D3D12_DEFAULT_SAMPLE_MASK;

    dxCommon_->GetDevice()->CreateGraphicsPipelineState(&pso, IID_PPV_ARGS(&pipelineState_));
}

// =========================================================
// パラメータをGPUに転送
// =========================================================
void PostProcess::UpdateBlurWeights() {
    BloomParam* p = nullptr;
    cbBloom_->Map(0, nullptr, reinterpret_cast<void**>(&p));
    *p = bloomParam_;
    cbBloom_->Unmap(0, nullptr);
}

// =========================================================
// 描画（動作確認済みの色反転版と同一構造 + CBV追加）
// =========================================================
void PostProcess::Draw(D3D12_GPU_DESCRIPTOR_HANDLE srcTextureHandle, ID3D12Resource* srcResource) {
    auto* cmd = dxCommon_->GetCommandList();

    auto Barrier = [&](ID3D12Resource* res, D3D12_RESOURCE_STATES from, D3D12_RESOURCE_STATES to) {
        D3D12_RESOURCE_BARRIER b{};
        b.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        b.Transition.pResource   = res;
        b.Transition.StateBefore = from;
        b.Transition.StateAfter  = to;
        cmd->ResourceBarrier(1, &b);
    };

    // 元画像を読み取り可能にする
    Barrier(srcResource, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_GENERIC_READ);

    // 出力テクスチャに描画
    Barrier(texOutput_.Get(), D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_RENDER_TARGET);
    auto rtv = dxCommon_->GetRTVHandle(outputRtvIndex_);
    float clearColor[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    cmd->OMSetRenderTargets(1, &rtv, false, nullptr);
    cmd->ClearRenderTargetView(rtv, clearColor, 0, nullptr);

    D3D12_VIEWPORT vp{ 0, 0, 1280.0f, 720.0f, 0, 1 };
    D3D12_RECT sc{ 0, 0, 1280, 720 };
    cmd->RSSetViewports(1, &vp);
    cmd->RSSetScissorRects(1, &sc);
    cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    cmd->SetGraphicsRootSignature(rootSignature_.Get());
    cmd->SetPipelineState(pipelineState_.Get());
    cmd->SetGraphicsRootDescriptorTable(0, srcTextureHandle);                    // t0: 元画像
    cmd->SetGraphicsRootConstantBufferView(1, cbBloom_->GetGPUVirtualAddress()); // b0: ブルームパラメータ
    cmd->DrawInstanced(3, 1, 0, 0);

    // 出力テクスチャを読み取り可能に戻す
    Barrier(texOutput_.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_GENERIC_READ);
}
