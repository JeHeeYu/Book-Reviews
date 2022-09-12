package com.example.myapplication

import android.os.Bundle
import android.os.SystemClock
import android.view.KeyEvent
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.example.myapplication.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {
    // 뒤로가기 버튼을 누른 시각을 저장하는 속성
    var initTime = 0L

    // 멈춘 시간을 저장하는 속성
    var pauseTime = 0L

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // start 버튼이 눌렸을 때
        binding.startButton.setOnClickListener {
            binding.chronometer.base = SystemClock.elapsedRealtime() + pauseTime
            binding.chronometer.start()

            // 버튼 표시 여부 조정
            binding.stopButton.isEnabled = true
            binding.resetButton.isEnabled = true
            binding.startButton.isEnabled = false
        }

        // stop 버튼이 눌렸을 때
        binding.stopButton.setOnClickListener {
            pauseTime = binding.chronometer.base - SystemClock.elapsedRealtime()
            binding.chronometer.stop()

            // 버튼 표시 여부 조정
            binding.stopButton.isEnabled = false
            binding.resetButton.isEnabled = true
            binding.startButton.isEnabled = true
        }

        // reset 버튼이 눌렸을 떄
        binding.resetButton.setOnClickListener {
            pauseTime = 0L

            binding.chronometer.base = SystemClock.elapsedRealtime()
            binding.chronometer.stop()

            // 버튼 표시 여부 조정
            binding.stopButton.isEnabled = false
            binding.resetButton.isEnabled = false
            binding.startButton.isEnabled = true
        }
    }


    //  뒤로가기 버튼 이벤트 핸들러
    override fun onKeyDown(keyCode: Int, event: KeyEvent?): Boolean {
        // 뒤로가기 버튼이 눌렸을 때 처리
        if (keyCode === KeyEvent.KEYCODE_BACK) {
            if(System.currentTimeMillis() - initTime > 3000) {
                Toast.makeText(this, "종료하려면 한번 더 누르세요!!", Toast.LENGTH_SHORT).show()
                initTime = System.currentTimeMillis()

                return true
            }
        }

        return super.onKeyDown(keyCode, event)
    }
}
