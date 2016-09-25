
#include "NsDirFilterG.h"


NsDirFilterGClass::NsDirFilterGClass(void)
{
}

void NsDirFilterGClass::init(int numGrid, float errorDir, float sigmaDir, float sigmaDirAV, float cpsDec)
{
  numGrid_    = numGrid;
  errorDir_   = errorDir;
  sigmaDir_   = sigmaDir;
  sigmaDirAV_ = sigmaDirAV;
  cpsDec_     = cpsDec;

  pr_     = (float*)malloc(sizeof(float) * numGrid_);
  pr_tmp_ = (float*)malloc(sizeof(float) * numGrid_);
  tr_     = (float*)malloc(sizeof(float) * numGrid_);
  for( int i=0; i<numGrid_; i++){
    pr_[i]     = 1.0f / numGrid_;
    pr_tmp_[i] = 0.0f;
    tr_[i]     = 0.0f;
  }
  tr_[0] = 1.0f;
  timePrev_ = 0;
  dirAVPrev_ = 0.0f;
  dir_ = -1.0f;
}

// 予測(ジャイロの観測量に従って確率分布を更新, 3*sigmaの範囲内のみで近似計算)
void NsDirFilterGClass::predict(unsigned long timeNow, float dirAV)
{
  if(timePrev_ > 0){
    // 状態遷移確率テーブルを設定(3*sigmaの範囲内のみ)
    float ddir  = (dirAV + dirAVPrev_) * (timeNow - timePrev_) / 1000.0f / 2.0f;
    float sigma = sigmaDirAV_ * (timeNow - timePrev_) / 1000.0f;
    int idx_min = floor((ddir - 3 * sigma) * numGrid_ / 360.0f);
    int idx_max = ceil((ddir + 3 * sigma) * numGrid_ / 360.0f);
    for(int j=idx_min; j<=idx_max; j++){
      int jj = j;
      if (jj < 0) {
        jj += numGrid_;
      } else if (jj >= numGrid_) {
        jj -= numGrid_;
      }
      float dir = jj * 360.0f / numGrid_;
      float d = min(abs(ddir - dir), abs(360.0f - max(ddir, dir) + min(ddir, dir)));
      tr_[jj] = exp(-1.0f * d * d / 2.0f / sigma / sigma);
    }
    // 確率分布に状態遷移確率テーブルを畳み込み
    float prsum = 0.0f;
    for(int i=0; i<numGrid_; i++){
      for(int j=idx_min; j<=idx_max; j++){
        int jj = (j < 0 ? j + numGrid_ : (j >= numGrid_ ? j - numGrid_ : j));
        int idx = i + jj;
        if(idx >= numGrid_) idx -= numGrid_;
        float tmp = pr_[i] * tr_[jj];
        pr_tmp_[idx] += tmp;
        prsum += tmp;
      }
    }
    // 確率分布を正規化＆一時配列を初期化
    if(prsum > 0.0){
      for(int i=0; i<numGrid_; i++){
        pr_[i] = pr_tmp_[i] / prsum;
        pr_tmp_[i] = 0.0f;
        tr_[i] = 0.0f;
      }
    } else {
      for(int i=0; i<numGrid_; i++){
        pr_[i] = 1.0f / numGrid_;
        pr_tmp_[i] = 0.0f;
        tr_[i] = 0.0f;
      }
    }
  }
  timePrev_ = timeNow;
  dirAVPrev_ = dirAV;
}

// 推定(コンパスの観測量に従って確率分布を更新, 3*sigmaの範囲のみで近似計算)
float NsDirFilterGClass::estimate(int dirId)
{
  // 確率分布を更新(3*sigmaの範囲内のみ)
  float dir = dirId * 45.0f + 22.5f;
  int idx_min = floor((dir - 22.5f - errorDir_ - 3.0f * sigmaDir_) * numGrid_ / 360.0f);
  int idx_max = ceil((dir + 22.5f + errorDir_ + 3.0f * sigmaDir_) * numGrid_ / 360.0f);
  float prsum = 0.0f;
  for(int i=idx_min; i<=idx_max; i++){
    int ii = (i < 0 ? i + numGrid_ : (i >= numGrid_ ? i - numGrid_ : i));
    float di = ii * 360.0f / numGrid_;
    float d = min(abs(di - dir), abs(360.0f - max(di, dir) + min(di, dir)));
    if(d < 22.5f + errorDir_) {
      pr_tmp_[ii] = pr_[ii];
    } else {
      pr_tmp_[ii] = pr_[ii] * exp(-1.0f * (d - 22.5f - errorDir_) * (d - 22.5f - errorDir_) / 2.0f / sigmaDir_ / sigmaDir_);
    }
    prsum += pr_tmp_[ii];
  }
  // 確率分布を正規化
  float maxPr = -1.0f;
  float maxIdx = -1;
  if(prsum > 0.0){
    for(int i=0; i<numGrid_; i++){
      pr_[i] = pr_tmp_[i] / prsum;
      pr_tmp_[i] = 0.0f;
      if(pr_[i] > maxPr){
        maxPr = pr_[i];
        maxIdx = i;
      }
    }
    dir_ = maxIdx * 360.0f / numGrid_ + cpsDec_;
    if(dir_ >= 360.0f) dir_ -= 360.0f;
  } else {
    for(int i=0; i<numGrid_; i++){
      pr_[i] = 1.0f / numGrid_;
      pr_tmp_[i] = 0.0f;
    }
    dir_ = -1.0f;
  }
  return dir_;
}

float NsDirFilterGClass::getDir()
{
  return dir_;
}


NsDirFilterGClass NsDirFilterG;
