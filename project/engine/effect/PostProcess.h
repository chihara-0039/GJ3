#pragma once
#include "base/DirectXCommon.h"
#include "base/SrvManager.h"
#include <wrl.h>
#include <d3d12.h>

/**
 * PostProcessクラス（単一パス版）
 * 動作確認済みの1パス構造をベースに、ブルームを実装する。
 * 1枚のシェーダー内で「高輝度抽出 + ぼかし + 合成」をすべて行う。
 */
class PostProcess {
public:
    // ブルームパラメータ（定数バッファとしてGPUに転送する）
    struct BloomParam {
        float threshold  = 0.5f; // この輝度以上が光る
        float intensity  = 1.5f; // 光の強さ
        float blurRadius = 4.0f; // ぼかし半径（ピクセル単位）
        float padding    = 0.0f;
    };

    void Initialize(DirectXCommon* dxCommon, SrvManager* srvManager);
    void Draw(D3D12_GPU_DESCRIPTOR_HANDLE srcTextureHandle, ID3D12Resource* srcResource);

    // ImGuiから調整するためのアクセサ
    BloomParam& GetBloomParam() { return bloomParam_; }
    float* GetBlurSigma() { return &bloomParam_.blurRadius; } // 互換性のため
    void UpdateBlurWeights(); // パラメータをGPUに転送

    // 最終出力テクスチャのSRVインデックス（ImGui表示用）
    uint32_t GetFinalSrvIndex() const { return outputSrvIndex_; }

private:
    DirectXCommon* dxCommon_ = nullptr;
    SrvManager* srvManager_ = nullptr;

    // 出力テクスチャ（1枚）
    Microsoft::WRL::ComPtr<ID3D12Resource> texOutput_;
    uint32_t outputRtvIndex_ = 0;
    uint32_t outputSrvIndex_ = 0;

    // パイプライン
    Microsoft::WRL::ComPtr<ID3D12RootSignature> rootSignature_;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> pipelineState_;

    // 定数バッファ
    Microsoft::WRL::ComPtr<ID3D12Resource> cbBloom_;

    // パラメータ
    BloomParam bloomParam_;
};