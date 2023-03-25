package com.example.myapplication

import android.os.Bundle
import android.view.View
import androidx.appcompat.app.AppCompatActivity
import com.example.myapplication.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // BUTTON2만 출력
        binding.btn1.setOnClickListener {
            binding.btn1.visibility = View.INVISIBLE
            binding.btn2.visibility = View.VISIBLE
        }

        // BUTTON1만 출력
        binding.btn2.setOnClickListener {
            binding.btn1.visibility = View.VISIBLE
            binding.btn2.visibility = View.INVISIBLE
        }

    }
}
