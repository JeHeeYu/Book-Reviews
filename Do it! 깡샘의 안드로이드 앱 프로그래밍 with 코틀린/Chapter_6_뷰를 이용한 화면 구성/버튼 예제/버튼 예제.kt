package com.example.myapplication

import android.graphics.Typeface
import android.os.Bundle
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // 이름 문자열 출력 TextView 속성
        val name = TextView(this).apply {
            typeface = Typeface.DEFAULT_BOLD
        }
        setContentView(R.layout.activity_main)
    }
}
