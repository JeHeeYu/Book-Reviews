package com.example.myapplication

import android.graphics.Typeface
import android.os.Bundle
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val name = TextView(this).apply {
            typeface = Typeface.DEFAULT_BOLD
        }
        setContentView(R.layout.activity_main)
    }
}
