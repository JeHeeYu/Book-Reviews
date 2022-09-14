package com.example.myapplication

import android.os.Bundle
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import com.example.myapplication.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val dialogBinding = ActivityMainBinding.inflate(layoutInflater)
        AlertDialog.Builder(this).run {
            setTitle("Input")
            setView(dialogBinding.root)
            setPositiveButton("닫기", null)
            show()
        }
    }
}
